import numpy as np
from opendbc.can.packer import CANPacker
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, apply_std_steer_angle_limits, apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.values import CANBUS, CarControllerParams, SteerDirection
from opendbc.car.volvo.volvocan import create_button_msg, create_lka_msg, create_lkas_state_msg, create_longitudinal, create_radar


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.acc_check = 1

    self.apply_torque_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # Cancel ACC if engaged when OP is not, but only above minimum steering speed.
    # TODO: is this check needed? it might trying to fix broken standstill behavior
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(create_button_msg(self.packer_pt, cancel=True))

    if self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        # calculate steer and also set limits due to driver torque
        new_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, CarControllerParams)
        apply_steer_dir = SteerDirection.LEFT if apply_torque > 0 else SteerDirection.RIGHT
      else:
        apply_torque = 0
        apply_steer_dir = SteerDirection.NONE

      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_msg(self.packer_pt, apply_torque, int(apply_steer_dir)))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, CS.ACC_Check))
      can_sends.append(create_radar(self.packer_pt, CS.stock_FSM1))

    # SNG
    # wait 100 cycles since last resume sent
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01 and not self.waiting:
        self.distance = CS.acc_distance
        #self.accel_status = accel
        self.waiting = True
        self.sng_count = 0
      if CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01 and self.waiting and CS.acc_distance > self.distance:
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([create_button_msg(self.packer_pt, resume=True)] * 25)
        can_sends.extend([create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, self.acc_check)] * 25)
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
