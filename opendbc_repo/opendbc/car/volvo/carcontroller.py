import numpy as np
from opendbc.can import CANPacker
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

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.takeoff_start_frame = -1_000_000  # frame when last SNG resume blast started
    self.op_standstill_frames = 0  # consecutive long_tx ticks stopped with longActive

    # virtual lead distance in FSM1; proportional to OP decel to grant ECU hydraulic-brake authority (drive 4d4)
    # starts at 255 ("no lead"), drifts ≤10 units/frame toward target to avoid step changes
    self.virt_dist = 255.0

    # wall-clock gate for FSM3/FSM1 TX — frame%2 unreliable due to controlsd jitter (drive 41)
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock

    self.sng_ack_frames = 0  # remaining FSM3 TXs with ACC_Check=1 during resume blast

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

        # virtual lead: knee at -0.15 m/s² → dist 80, saturates at -0.80 m/s² → dist 35
        if accel < -0.15:
          frac = min(1.0, (abs(accel) - 0.15) / 0.65)
          target_dist = 80.0 - frac * 45.0
        else:
          target_dist = 255.0
        self.virt_dist += max(-10.0, min(10.0, target_dist - self.virt_dist))
        virt_dist_int = int(round(self.virt_dist))
        virt_b1 = max(235, min(250, int(235 + (80 - virt_dist_int) * 0.333)))
        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, True,
                                               virt_dist=virt_dist_int, virt_b1=virt_b1))
      else:
        self.op_standstill_frames = 0
        acc_standstill = None  # pass stock ACC_Standstill through
        accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        if self.sng_ack_frames > 0:
          acc_check = 1
          self.sng_ack_frames -= 1
        else:
          acc_check = int(CS.stock_FSM3["ACC_Check"])
        # drift virt_dist back to 255 so the next longActive period starts neutral
        self.virt_dist = min(255.0, self.virt_dist + 10.0)
        can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, CC.longActive))

      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, acc_standstill))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
