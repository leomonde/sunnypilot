from opendbc.can import CANPacker
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CANBUS, CarControllerParams, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE

    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # Virtual lead smoothing state (oplong)
    # Rate-limit on accel (not dist/vLead separately) so FSM1/FSM3/FSM4 stay
    # mutually consistent: compute_virtual_lead derives dist and vLead from the
    # same rate-limited accel that goes into FSM3, eliminating the mismatch that
    # caused the ECM to kill ACC during re-engagement transitions.
    self._vl_accel_prev: float = 0.0   # m/s², tracks rate-limited accel

    # Minimum ego speed to activate virtual lead (30 km/h)
    self.OPLONG_MIN_SPEED_MS: float = 30.0 / 3.6
    # Max accel change per 20 ms tick: 0.5 m/s² → 25 m/s³ jerk limit
    self._VL_ACCEL_RATE: float = 0.5   # m/s² per tick

    # FSM3 Byte_01: speed-dependent follow mode (route 5a0 analysis).
    # 29 (0b11101) = FrontCar low-speed (<55 km/h); 21 (0b10101) = high-speed (>63 km/h).
    # 7 km/h hysteresis: hold previous value in 55–63 km/h band.
    self._vl_byte01: int = 29

    # FSM4 Byte_0: rolling 1-bit counter toggles 85↔170 every send (route 5a0 analysis).
    self._fsm4_toggle: int = 85

    # Custom ACC increment: read once at init, refreshed every 100 frames.
    # Passed to carstate so _pending_delta emits the right number of events.
    self._params = Params()
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    # FSM3 ACC_Check=1 ack window: stock FSM3 has ACC_Check=0; ECU ignores
    # OP's CCButtons resume unless FSM3 confirms it. Force 25 frames (~0.5s).
    self.sng_ack_frames = 0

    # wall-clock gate for FSM3/FSM1 TX — frame%2 unreliable due to controlsd jitter
    # complex fwd_hook blocks stock FSM3/FSM1 when controls_allowed && !gas_pressed,
    # so OP must relay them at 50Hz to prevent ECU faults.
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock FSM3/FSM1

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # Compute long_active once so both the FSM0 relay (100 Hz) and the
    # long_tx_due block (50 Hz) use the same value.
    long_active = CC.longActive and CS.out.vEgo >= self.OPLONG_MIN_SPEED_MS

    # Relay FSM0 only when virtual lead is active (needs ACC_FrontCar=1).
    # When oplong is disabled, OP does not touch FSM0 — same as before oplong
    # was introduced — so the ECU receives FSM0 unmodified from the camera.
    if long_active:
      can_sends.append(volvocan.create_fsm0(self.packer_pt, CS.stock_FSM0, True))

    # TODO: verify if this minSteerSpeed guard is still needed
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # run at 50hz
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        #apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CarControllerParams)
        apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        # Update prev with desired if just enabled.
        if not self.latActive_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
            self.steer_blocked = False
        else:
          if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
            self.steer_blocked = True
            self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
            self.steer_dir_bf_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          # Set old request when inside deadzone
          apply_steer_dir = self.apply_steer_dir_prev

      else:
        apply_steer = 0
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.latActive_prev = CC.latActive

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)

    # SNG
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      lead_moved = CS.acc_distance > self.distance

      if at_standstill and self.waiting and lead_moved:
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # FSM1/FSM3/FSM4 at 50 Hz wall-clock: fwd_hook blocks stock cam→main when
    # controls_allowed && !gas_pressed, so OP must relay all three to prevent faults.
    # Virtual lead is injected into FSM1/FSM4 when CC.longActive at ≥30 km/h.
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      if self.sng_ack_frames > 0:
        acc_check = 1
        self.sng_ack_frames -= 1
      else:
        acc_check = int(CS.stock_FSM3["ACC_Check"])

      vEgo_ms = CS.out.vEgo

      if long_active:
        # Rate-limit on accel so FSM1/FSM3/FSM4 are always derived from the
        # same value — previously dist and vLead were limited independently
        # while FSM3 sent the raw accel, causing ~2× mismatch during
        # re-engagement that made the Volvo ECM kill ACC.
        dt_accel = self._VL_ACCEL_RATE * 0.02
        accel_raw = float(actuators.accel)
        accel_limited = max(accel_raw, self._vl_accel_prev - dt_accel)
        accel_limited = min(accel_limited, self._vl_accel_prev + dt_accel)
        self._vl_accel_prev = accel_limited

        # Byte_01 speed-dependent mode with 7 km/h hysteresis (route 5a0 analysis):
        # 29 (FrontCar active, low speed) below 55 km/h; 21 (high speed) above 63 km/h.
        vEgo_kmh = vEgo_ms * 3.6
        if vEgo_kmh > 63.0:
          self._vl_byte01 = 21
        elif vEgo_kmh < 55.0:
          self._vl_byte01 = 29
        # else: hold previous value in 55–63 km/h hysteresis band

        # FSM4 Byte_0 rolling counter: toggle 85↔170 every send (route 5a0 analysis).
        self._fsm4_toggle = 170 if self._fsm4_toggle == 85 else 85

        # Compute virtual lead from rate-limited accel; dist/vLead now consistent with FSM3.
        vl = volvocan.compute_virtual_lead(accel_limited, vEgo_ms)

        # Safety: if the real FSM reports a lead car closer than our virtual,
        # always use the closer distance so the ACC brakes at least as hard.
        real_dist = float(CS.acc_distance)
        if real_dist > 0 and real_dist < vl['dist_virtual']:
          vl['dist_virtual'] = real_dist

        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, virtual_lead=vl))
        can_sends.append(volvocan.create_lead_speed(self.packer_pt, vl['vLead_kmh'], CS.stock_FSM4, byte0=self._fsm4_toggle))
        accel  = vl['accel_request']
        byte2  = vl['byte2_fsm3']
      else:
        # Not in oplong mode or below min speed: seed smoothing with stock accel
        # so the first active tick starts from a consistent value.
        self._vl_accel_prev = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1))
        # Relay FSM4 with all stock bytes — create_lead_speed hardcodes Byte_1/
        # Byte_4 for virtual-lead mode; those values cause ECU faults in normal
        # ACC mode (e.g. when ICBM is active with oplong disabled).
        can_sends.append(volvocan.create_fsm4_passthrough(self.packer_pt, CS.stock_FSM4))
        accel  = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        byte2  = None

      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, byte2=byte2, byte01=self._vl_byte01 if byte2 is not None else None))

    # Refresh custom ACC step every 100 frames and forward to carstate so that
    # _pending_delta emits exactly one synthetic event per physical ACC step.
    if self.frame % 100 == 0:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management — skip when oplong is active to avoid
    # synthetic button presses disrupting the virtual-lead longitudinal control.
    if not self.CP.openpilotLongitudinalControl:
      icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
      if icbm_sends:
        CS._pending_delta = 0
        CS._icbm_suppress_frames = 25
      can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
