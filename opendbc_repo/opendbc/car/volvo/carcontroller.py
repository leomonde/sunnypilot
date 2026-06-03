import cereal.messaging as messaging
from opendbc.can import CANPacker
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.volvo.sla import VolvoSlaController


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

    # Custom ACC increment (refreshed every 100 frames); used by carstate to size events.
    self._params = Params()
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    # SpeedLimitMode (UI toggle): 0=off, 1=info, 2=warning, 3=assist. Refreshed every 100 frames.
    self._sla_mode = int(self._params.get("SpeedLimitMode", return_default=True) or 0)

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.sng_ack_frames = 0  # ACC_Check=1 window so ECU acks OP's CCButtons resume

    # Wall-clock TX gates. Stock frequencies (000005ca): FSM1=FSM3=50Hz, FSM4=33Hz.
    # Sending FSM4 at 50Hz advances Heartbeat 50% too fast → ECM pcmDisable after ~7s.
    self.next_long_tx_nanos = 0
    self.next_fsm4_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz
    self.FSM4_TX_PERIOD_NANOS = 30_000_000  # 33Hz

    # Hysteresis flags (avoid ECM seeing rapid flips around thresholds).
    self.strong_braking = False  # FSM1 TargetState bit 2 — enter -0.40, exit -0.20
    self.vlc_active = False      # FSM3 B0 bit 6 — enter -0.55, exit -0.40
    # Hysteresis on lead presence: stay in Phase 2 (clamp/passthrough) until BOTH
    # stock_dist clears AND stock releases TargetState bit 2 (its strong-brake flag).
    # Prevents cross-msg incoherence when stock lags releasing brake flags after lead loss.
    self.has_real_lead_state = False

    # Auto-arming Volvo SLA via ICBM. Runs in parallel with mainline SLA (which is
    # neutralized for Volvo+oplong via the speed_limit_assist.py one-line patch).
    # Driver disarms via single press. Reset on next ACC engagement.
    self.sla = VolvoSlaController()
    # Subscribe to mainline's policy-resolved speed limit (TSR+map per UI policy:
    # SpeedLimitControlPolicy param decides TSR-first/map-first/TSR-only/map-only).
    # Falls back to raw TSR if SubMaster yields nothing (e.g., tests).
    self._sm_lp = messaging.SubMaster(['longitudinalPlanSP'])

    # Brake jerk limiter state — clamps negative deltas on TX'd op_accel.
    self.last_op_accel_tx = 0.0

    # Setpoint-drop softening: soften OP braking right after the setpoint is lowered
    # (e.g. SLA set-) to avoid the ECM rejecting ACC (pcmDisable) on a fast-dropping
    # setpoint + hard brake while the car is still well above the new setpoint.
    self._setpoint_kph_prev = 0.0
    self._setpoint_drop_frame = -100000

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
        apply_steer = apply_std_steer_angle_limits(
          actuators.steeringAngleDeg, self.apply_steer_prev,
          CS.out.vEgoRaw, CS.out.steeringAngleDeg,
          CC.latActive, CarControllerParams.ANGLE_LIMITS,
        )
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
        # Burst BUTTON_BURST identical frames to increase the likelihood of resume being accepted.
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * CarControllerParams.BUTTON_BURST)
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # FSM3/FSM1 at 50Hz and FSM4 at 33Hz: relay stock when oplong off; inject OP accel
    # otherwise. fwd_hook blocks stock FSM1/3/4 cam→main when controls_allowed.
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    fsm4_tx_due = now_nanos >= self.next_fsm4_tx_nanos
    if long_tx_due or fsm4_tx_due:
      v_ego_ms = CS.out.vEgoRaw
      # Phase 1 (no lead): OP fully controls — cruise reduction pattern (Distance=255).
      # Phase 2 (with lead): OP overrides brake params but lead data passes through;
      # accel is clamped within ±BRAKE_CLAMP_MARGIN of stock to avoid ECM rejection.
      # Sign-coherence: when stock is clearly positive (engagement transient, coming off
      # gas, etc.) and OP wants to brake, defer to stock to avoid cross-msg incoherence.
      stock_dist = CS.stock_FSM1["ACC_Distance"]
      stock_TS_bit2 = bool(int(CS.stock_FSM1["ACC_TargetState"]) & 0b100)
      if self.has_real_lead_state:
        # Stay in Phase 2 until stock both releases distance AND clears strong-brake bit.
        if stock_dist >= 220 and not stock_TS_bit2:
          self.has_real_lead_state = False
      else:
        if stock_dist < 200:
          self.has_real_lead_state = True
      stock_has_real_lead = self.has_real_lead_state
      op_active = self.CP.openpilotLongitudinalControl and CC.longActive
      stock_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      op_planner = float(actuators.accel) if op_active else stock_accel
      # Emergency brake: planner wants strong brake AND stock isn't already braking.
      # Bypass clamp + force hydraulic flags so the car can physically deliver, even
      # if stock isn't engaging. Gated by stock_accel > EMERGENCY_STOCK_PASSIVE to
      # avoid the worst cross-msg divergence (both braking but stock in B*, OP in F*).
      emergency_brake = (op_active
                         and op_planner < CarControllerParams.EMERGENCY_BRAKE_THRESHOLD
                         and stock_accel > CarControllerParams.EMERGENCY_STOCK_PASSIVE)
      # Sign coherence: defer to stock only when NOT in emergency.
      sign_mismatch = (not emergency_brake
                       and stock_accel > CarControllerParams.STOCK_POSITIVE_TRANSIENT
                       and op_planner < 0)
      op_controls_long = op_active and not sign_mismatch

      if op_controls_long:
        if stock_has_real_lead and not emergency_brake:
          op_accel = max(stock_accel - CarControllerParams.BRAKE_CLAMP_MARGIN,
                         min(op_planner, stock_accel + CarControllerParams.BRAKE_CLAMP_MARGIN))
        else:
          op_accel = op_planner  # free authority (no lead OR emergency)
      else:
        op_accel = stock_accel  # passthrough (op inactive or sign-mismatch defer)
      has_real_lead = stock_has_real_lead

      # Setpoint-drop softening: when the cruise setpoint was just lowered (e.g. SLA set-),
      # a large speed-vs-setpoint gap combined with an aggressive OP brake makes the ECM
      # reject ACC (pcmDisable → cruise fault, drive 0614 log1). While the setpoint is still
      # settling and there's no real lead / emergency, cap braking so the car coasts down
      # gently instead of out-braking the stock ACC during the transition.
      setpoint_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
      if setpoint_kph < self._setpoint_kph_prev - 0.5:
        self._setpoint_drop_frame = self.frame
      self._setpoint_kph_prev = setpoint_kph
      in_setpoint_transition = (self.frame - self._setpoint_drop_frame) < CarControllerParams.SETPOINT_TRANSITION_HOLD
      if op_controls_long and not has_real_lead and not emergency_brake and in_setpoint_transition:
        op_accel = max(op_accel, CarControllerParams.SETPOINT_TRANSITION_BRAKE)

      # Phase 1b: ACC_Speed coherence guard. Suppress OP brake near setpoint ONLY when
      # stock isn't actively braking — stock_accel is the proxy for lead/AEB dominance.
      # Stock combines setpoint + lead logic; if it asks for real brake, lead is dominant.
      if op_active and op_accel < 0:
        acc_target_ms = CS.out.cruiseState.speed
        near_setpoint = abs(v_ego_ms - acc_target_ms) < CarControllerParams.ACC_SPEED_COHERENCE_MARGIN
        stock_braking = stock_accel < CarControllerParams.STOCK_BRAKE_DEMAND
        if near_setpoint and not stock_braking:
          op_accel = 0.0

      # Brake jerk limit: clamp NEGATIVE deltas so brake ramps in progressively.
      # Releasing brake (positive delta) is free. Fixes step transitions like
      # +0.96 → -2.04 when emergency activates after Phase 1b suppression
      # (route 0000060b log 9 t=560.25). Applied at TX rate (~50Hz here).
      if op_active:
        dt = self.LONG_TX_PERIOD_NANOS / 1e9  # ~0.02s @ 50Hz
        max_decrement = CarControllerParams.MAX_BRAKE_JERK * dt
        op_accel = max(op_accel, self.last_op_accel_tx - max_decrement)
      self.last_op_accel_tx = op_accel

      # vlc_active hysteresis (enter -0.55, exit -0.40); abaixo de 30 km/h → passthrough.
      if op_controls_long and v_ego_ms >= (30.0 / 3.6):
        if self.vlc_active:
          self.vlc_active = op_accel < volvocan._VLC_EXIT_THRESHOLD
        else:
          self.vlc_active = op_accel < volvocan._VLC_ENTER_THRESHOLD
      else:
        self.vlc_active = False

      # strong_braking hysteresis (enter -0.40, exit -0.20).
      if self.strong_braking:
        self.strong_braking = op_accel < -0.20
      else:
        self.strong_braking = op_accel < -0.40

    # 33Hz — FSM4
    if fsm4_tx_due:
      next_fsm4 = self.next_fsm4_tx_nanos + self.FSM4_TX_PERIOD_NANOS
      if self.next_fsm4_tx_nanos == 0 or next_fsm4 <= now_nanos:
        next_fsm4 = now_nanos + self.FSM4_TX_PERIOD_NANOS
      self.next_fsm4_tx_nanos = next_fsm4

      can_sends.append(volvocan.create_fsm4(self.packer_pt, CS.stock_FSM4,
                                            op_controls_long, op_accel, v_ego_ms,
                                            self.strong_braking, has_real_lead,
                                            emergency_brake))

    # 50Hz — FSM1 + FSM3
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
                                             op_controls_long, self.strong_braking, has_real_lead,
                                             emergency_brake))
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3,
                                                    op_accel, acc_check, self.vlc_active,
                                                    has_real_lead, emergency_brake))

    # Refresh custom ACC step every 100 frames; carstate uses it to size pending_delta events.
    if self.frame % 100 == 0:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
      self._sla_mode = int(self._params.get("SpeedLimitMode", return_default=True) or 0)
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management (only runs when oplong is off; bypassed here)
    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
    if icbm_sends:
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    # Volvo SLA via ICBM (auto-arm, in-opendbc). Skip if ICBM already pressed this frame.
    if not icbm_sends:
      # Read mainline's policy-resolved speed limit (TSR+map, ordered by user's
      # SpeedLimitControlPolicy param in the UI). Falls back to raw TSR if no LP_SP yet.
      self._sm_lp.update(0)
      resolved_kph = 0.0
      if self._sm_lp.seen['longitudinalPlanSP']:
        resolved_kph = float(self._sm_lp['longitudinalPlanSP'].speedLimit.resolver.speedLimit) * CV.MS_TO_KPH
      speed_limit_kph = resolved_kph if resolved_kph > 0 else CS.tsr_speed_kph

      sla_action = self.sla.update(
        tsr_kph=speed_limit_kph,
        setpoint_kph=CS.out.cruiseState.speed * CV.MS_TO_KPH,
        vEgo_kph=CS.out.vEgoRaw * CV.MS_TO_KPH,
        acc_on=CS.out.cruiseState.enabled,
        frame=self.frame,
        sla_mode=self._sla_mode,
        driver_adjust_press=CS.driver_btn_adjust,
        driver_resume_press=CS.driver_btn_resume,
      )
      if sla_action == 'set-':
        # Burst BUTTON_BURST frames so the ACC registers the press (a single frame is ignored).
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, minus=True)] * CarControllerParams.BUTTON_BURST)
        CS._icbm_suppress_frames = 25  # mute synthetic event in carstate
      elif sla_action == 'set+':
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, set_plus=True)] * CarControllerParams.BUTTON_BURST)
        CS._icbm_suppress_frames = 25

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
