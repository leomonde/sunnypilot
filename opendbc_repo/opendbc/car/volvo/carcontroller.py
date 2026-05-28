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

    # wall-clock gate for FSM3/FSM1/FSM4 TX — frame%2 unreliable due to controlsd jitter.
    # fwd_hook blocks stock FSM1/FSM3/FSM4 from cam→main when controls_allowed && !gas_pressed,
    # so OP must relay them itself to prevent ECU faults.
    # Stock TX frequencies (measured on 000005ca seg 6, 60s): FSM1=50Hz, FSM3=50Hz, FSM4=33Hz.
    # Sending FSM4 at 50Hz causes the rolling Heartbeat (Byte_0, 9-9-12 frame runs) to
    # advance 50% faster than the ECM expects, likely causing the post-7s pcmDisable
    # observed in 000005cd--465a34c797 seg 4.
    self.next_long_tx_nanos = 0
    self.next_fsm4_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock FSM3/FSM1
    self.FSM4_TX_PERIOD_NANOS = 30_000_000  # 33Hz, matching stock FSM4 cadence

    # Hysteresis for "strong braking" flag (FSM1 ACC_TargetState bit 2 and FSM4 BrakingMode).
    # Avoids rapid flips around accel ≈ -0.32 m/s² that confuse the ECM.
    self.strong_braking = False

    # VLC activation with hysteresis on accel (avoid flicker around -0.5 boundary).
    # VLC = inject close lead so ECM authorizes hydraulic braking. Only used when
    # OP commands moderate-to-strong braking; cruise/acceleration/light braking
    # run without VLC (stock accepts AccelRequest without lead in those cases).
    self.vlc_active = False

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

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

    # FSM3/FSM1 at 50Hz and FSM4 at 33Hz wall-clock: relay stock values so ECU doesn't fault.
    # (fwd_hook blocks stock FSM1/FSM3/FSM4 when controls_allowed && !gas_pressed)
    # When OP controls long: inject OP's accel and virtual lead car (FSM1+FSM4).
    # Override ACC_Check=1 during SNG resume blast so ECU acknowledges OP's CCButtons resume.
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    fsm4_tx_due = now_nanos >= self.next_fsm4_tx_nanos
    if long_tx_due or fsm4_tx_due:
      v_ego_ms = CS.out.vEgoRaw
      # CC.longActive alone is not enough: with pcmCruise=True, controlsd sets longActive
      # whenever cruise is enabled (ACC nativo ativo), regardless of who controls long.
      # CP.openpilotLongitudinalControl is the user toggle ("alpha longitudinal").
      # When False (oplong off + ICBM/SLA only), OP must NOT inject VLC — pure passthrough.
      op_controls_long = self.CP.openpilotLongitudinalControl and CC.longActive
      op_accel = float(actuators.accel) if op_controls_long else float(CS.stock_FSM3["ACC_AccelerationRequest"])

      # VLC activation hysteresis. Enter at -0.55, exit at -0.40. Only relevant if
      # OP controls long. Below 30 km/h relay stock (no VLC) since SNG/low-speed
      # following uses actual camera data.
      if op_controls_long and v_ego_ms >= (30.0 / 3.6):
        if self.vlc_active:
          self.vlc_active = op_accel < volvocan._VLC_EXIT_THRESHOLD
        else:
          self.vlc_active = op_accel < volvocan._VLC_ENTER_THRESHOLD
      else:
        self.vlc_active = False

      # Hysteresis on strong-braking flag (FSM1 ACC_TargetState bit 2, FSM4 BrakingMode).
      # Enter at -0.40 m/s², exit at -0.20 m/s² — avoids ECM seeing rapid flips around -0.32.
      if self.strong_braking:
        self.strong_braking = op_accel < -0.20
      else:
        self.strong_braking = op_accel < -0.40

    # 33Hz block — FSM4 (matches stock cam rate)
    if fsm4_tx_due:
      next_fsm4 = self.next_fsm4_tx_nanos + self.FSM4_TX_PERIOD_NANOS
      if self.next_fsm4_tx_nanos == 0 or next_fsm4 <= now_nanos:
        next_fsm4 = now_nanos + self.FSM4_TX_PERIOD_NANOS
      self.next_fsm4_tx_nanos = next_fsm4

      can_sends.append(volvocan.create_fsm4(self.packer_pt, CS.stock_FSM4,
                                            op_controls_long, self.vlc_active,
                                            op_accel, v_ego_ms, self.strong_braking))

    # 50Hz block — FSM1 + FSM3
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

      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1,
                                             op_controls_long, self.vlc_active,
                                             op_accel, v_ego_ms, self.strong_braking))
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3,
                                                    op_accel, acc_check, self.vlc_active))

    # Refresh custom ACC step every 100 frames and forward to carstate so that
    # _pending_delta emits exactly one synthetic event per physical ACC step.
    if self.frame % 100 == 0:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management
    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
    if icbm_sends:
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
