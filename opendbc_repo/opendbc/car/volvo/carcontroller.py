import numpy as np
from opendbc.can import CANPacker
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
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

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.takeoff_start_frame = -1_000_000  # frame when last SNG resume blast started
    self.op_standstill_frames = 0  # consecutive long_tx ticks stopped with longActive

    # wall-clock gate for FSM3/FSM1 TX — frame%2 unreliable due to controlsd jitter (drive 41)
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock FSM3/FSM1

    # wall-clock gate for FSM4 TX — stock cam sends FSM4 at 33Hz, not 50Hz (drive 4e1)
    self.next_fsm4_tx_nanos = 0
    self.FSM4_TX_PERIOD_NANOS = 30_000_000  # 33Hz, matching stock
    self.last_op_accel = 0.0  # last computed accel, used by 33Hz FSM4 block

    self.sng_ack_frames = 0  # remaining FSM3 TXs with ACC_Check=1 during resume blast

    # Custom ACC increment — read once at startup; refreshed periodically in update()
    self._params = Params()
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    accel = 0.0  # always defined so SNG block never NameErrors

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

    # With OP long-control, stock ACC never sets ACC_Standstill; use vehicle motion + 3-tick delay
    # so the ECU sees ACC_Standstill=1 in FSM3 before the resume blast fires (drive 38).
    if self.CP.openpilotLongitudinalControl and CC.longActive:
      at_standstill = (CS.out.cruiseState.enabled and CS.out.standstill
                       and self.op_standstill_frames >= 3)
    else:
      at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                       and CS.out.vEgo < 0.01)

    # SNG — evaluated before long-control TX so takeoff flag is visible below
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      lead_moved = CS.acc_distance > self.distance
      e2e_resume = CC.cruiseControl.resume

      if at_standstill and self.waiting and (lead_moved or e2e_resume):
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        if self.sng_count == 0:
          self.takeoff_start_frame = self.frame
          self.sng_ack_frames = 25
        self.sng_count += 1
      # exit waiting after 5 blasts or when car moves; OP long uses vehicle motion, not stock ACC_Standstill
      if self.CP.openpilotLongitudinalControl and CC.longActive:
        sng_exit = self.sng_count >= 5 or not CS.out.standstill
      else:
        sng_exit = self.sng_count >= 5 or not CS.out.cruiseState.standstill
      if self.waiting and sng_exit:
        self.waiting = False
        self.last_resume_frame = self.frame

    # FSM3/FSM1 at 50Hz wall-clock: OP accel when long active, stock passthrough otherwise (silence faults ECU)
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      if self.CP.openpilotLongitudinalControl and CC.longActive:
        op_accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

        # take-off passthrough: use stock's accel when it's higher than OP's to match ECM expectations (drive 42)
        takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
        stock_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        in_takeoff_window = takeoff_elapsed < 15.0 and CS.out.vEgo < 5.0  # 15s ceiling prevents runaway stock accel
        if in_takeoff_window and stock_accel > op_accel and stock_accel > 0:
          accel = stock_accel
        else:
          accel = op_accel

        # hold accel=0 at standstill until the resume blast; counter resets only on motion,
        # so ACC_Standstill stays asserted through the take-off window.
        if CS.out.standstill:
          self.op_standstill_frames += 1
          if not in_takeoff_window:
            accel = 0.0  # hold at exact-zero until the ECU gets the resume button
        else:
          self.op_standstill_frames = 0

        # assert only when truly stopped and not in takeoff window — vEgo gate prevents jerk; takeoff gate releases brake hold
        acc_standstill = 1 if (CS.out.standstill and self.op_standstill_frames >= 3 and abs(CS.out.vEgo) < 0.01 and not in_takeoff_window) else 0

        acc_check = 1 if self.sng_ack_frames > 0 else 0
        if self.sng_ack_frames > 0:
          self.sng_ack_frames -= 1

        # Persistent virtual lead — always active when longActive and no real lead.
        # Replicates stock FSM behavior: lead at 1.5s headway, speed = ego + accel*1.5s lookahead.
        # Lead is always present (never ON/OFF) so ECM never sees a lead appear/disappear.
        # hard_brake: set FSM1.tgt=0xBC when braking hard (stock confirmed drive 589 seg22:
        #   dist<20m + accel<-0.5 m/s²). Signals ECM that lead is braking → hydraulic brakes.
        virt_dist_m = max(10.0, CS.out.vEgo * 1.5)
        hard_brake = accel < -0.5
        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, True,
                                               virt_dist=int(round(virt_dist_m)), virt_b1=0xFF,
                                               hard_brake=hard_brake))
      else:
        self.op_standstill_frames = 0
        acc_standstill = None  # pass stock ACC_Standstill through
        accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        if self.sng_ack_frames > 0:
          acc_check = 1
          self.sng_ack_frames -= 1
        else:
          acc_check = int(CS.stock_FSM3["ACC_Check"])
        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, False))

      # store accel for use by 33Hz FSM4 block below
      self.last_op_accel = accel

      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, acc_standstill))

    # FSM4 virtual lead speed at 33Hz — matches stock cam rate to avoid ECM frequency faults (drive 4e1)
    # Decoupled from FSM3/FSM1 50Hz timer; uses last_op_accel updated each long_tx tick.
    fsm4_tx_due = now_nanos >= self.next_fsm4_tx_nanos
    if fsm4_tx_due:
      next_fsm4 = self.next_fsm4_tx_nanos + self.FSM4_TX_PERIOD_NANOS
      if self.next_fsm4_tx_nanos == 0 or next_fsm4 <= now_nanos:
        next_fsm4 = now_nanos + self.FSM4_TX_PERIOD_NANOS
      self.next_fsm4_tx_nanos = next_fsm4

      # FSM4 virtual lead — always active when longActive and no real lead.
      # Lead speed = ego + accel*1.5s lookahead (mirrors FSM1 logic above).
      # Byte_2: TTC×10 when closing (ego faster), else empirical from log analysis (ego_kmh×1.15≈70 at 60km/h).
      # brake_b5: B5=0xF3 when hard braking (stock confirmed drive 589 seg22: accel<-0.5 m/s²).
      #   B5=0xF2 for very hard braking (accel<-1.0). Required for ECM hydraulic brake activation.
      no_real_lead = int(CS.stock_FSM1["ACC_Distance"]) >= 200
      if self.CP.openpilotLongitudinalControl and CC.longActive and no_real_lead:
        virt_lead_ms = max(0.5, CS.out.vEgo + self.last_op_accel * 1.5)
        virt_lead_kmh = virt_lead_ms * CV.MS_TO_KPH
        virt_dist_m = max(10.0, CS.out.vEgo * 1.5)
        closing_ms = max(0.0, CS.out.vEgo - virt_lead_ms)
        if closing_ms > 0.1:
          virt_b2 = min(127, int(virt_dist_m / closing_ms * 10))
        else:
          virt_b2 = min(127, int(virt_lead_kmh * 1.15))
        # brake_b5 mirrors stock FSM4.Byte_5 pattern: 0xF2=very hard, 0xF3=hard, None=normal
        if self.last_op_accel < -1.0:
          brake_b5 = 0xf2
        elif self.last_op_accel < -0.5:
          brake_b5 = 0xf3
        else:
          brake_b5 = None
        can_sends.append(volvocan.create_fsm4(self.packer_pt, CS.stock_FSM4, virt_lead_kmh,
                                              virt_b2=virt_b2, brake_b5=brake_b5))
      else:
        can_sends.append(volvocan.create_fsm4(self.packer_pt, CS.stock_FSM4))

    # FSM0 at 100Hz — same rate as stock cam. Override ACC_FrontCar=1 whenever OP has
    # an active virtual lead (FSM1 virt_dist < 255), so ECM grants hydraulic-brake authority.
    # Byte_0 (rolling counter) and Byte_7 (checksum) are passed from stock unchanged —
    # log analysis confirmed byte7 does not cover byte2, so ACC_FrontCar override is checksum-safe.
    virt_lead_active = (self.CP.openpilotLongitudinalControl and CC.longActive
                        and int(CS.stock_FSM1["ACC_Distance"]) >= 200)
    can_sends.append(volvocan.create_fsm0(self.packer_pt, CS.stock_FSM0,
                                          front_car_override=1 if virt_lead_active else None))

    # Refresh custom ACC step from params every ~100 frames (~1 s) and forward to carstate
    if self.frame % 100 == 0:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management
    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
    if icbm_sends:
      # Suppress synthetic _pending_delta events caused by ICBM's own ACC speed change
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
