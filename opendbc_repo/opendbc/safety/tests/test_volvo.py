#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):

  TX_MSGS = [[0x127, 0], [0x246, 2], [0x051, 0], [0x262, 0], [0x260, 0], [0x270, 0], [0x31A, 0]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: [0x051, 0x262, 0x31A], 2: [0x246]}
  FWD_BLACKLISTED_ADDRS = {2: [0x051, 0x262, 0x31A], 0: [0x246]}

  VOLVO_MAIN_BUS = 0
  #VOLVO_AUX_BUS = 1
  VOLVO_CAM_BUS = 2

  # Angle control limits
  STEER_ANGLE_MAX = 45  # deg, reasonable limit
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"LKAAngleReq": angle, "LKASteerDirection": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("FSM2", self.VOLVO_MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"SteeringAngleServo": angle}
    return self.packer.make_can_msg_safety("PSCM1", self.VOLVO_MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_Enabled": 1 if enable else 0}
    return self.packer.make_can_msg_safety("FSM0", self.VOLVO_CAM_BUS, values)

  def _speed_msg(self, speed: float):
    values = {"VehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("VehicleSpeed1", self.VOLVO_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePedal": 2 if brake else 0}
    return self.packer.make_can_msg_safety("Brake_Info", self.VOLVO_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"AccPedal": 10 if gas else 0}
    return self.packer.make_can_msg_safety("AccPedal", self.VOLVO_MAIN_BUS, values)
  
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehicleSpeed": 0 if speed <= self.STANDSTILL_THRESHOLD else 10}
    return self.packer.make_can_msg_safety("VehicleSpeed1", 0, values)

  def test_angle_cmd_when_disabled(self):
    # Volvo uses inactive_angle_is_zero=True: ECU ignores LKAAngleReq when
    # LKASteerDirection=0, and carcontroller always sends angle=0 when inactive.
    # Only angle=0 is allowed in inactive mode, regardless of angle_meas.
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for steer_control_enabled in (True, False):
        for angle_meas in np.arange(-90, 91, 10):
          self._reset_angle_measurement(angle_meas)
          for angle_cmd in np.arange(-90, 91, 10):
            self._set_prev_desired_angle(angle_cmd)
            if steer_control_enabled:
              should_tx = controls_allowed
            else:
              should_tx = (angle_cmd == 0)
            self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_cmd, steer_control_enabled)))

  def test_angle_cmd_when_enabled(self):
    # Rate-limit tests when active, plus Volvo-specific inactive behavior.
    speeds = [0., 1., 5., 10., 15., 50.]
    steer_angle_test_max = self.STEER_ANGLE_MAX * 2
    angles = np.concatenate((np.arange(-steer_angle_test_max, steer_angle_test_max, 5), [0]))
    for a in angles:
      for s in speeds:
        max_delta_up = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP)
        max_delta_down = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN)

        self._reset_angle_measurement(a)
        self._reset_speed_measurement(s)
        self._set_prev_desired_angle(a)
        self.safety.set_controls_allowed(1)

        self.assertTrue(self._tx(self._angle_cmd_msg(a + common.sign_of(a) * max_delta_up, True)))
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertTrue(self._tx(self._angle_cmd_msg(a - common.sign_of(a) * max_delta_down, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        self.assertFalse(self._tx(self._angle_cmd_msg(a + common.sign_of(a) * (max_delta_up + 1.1), True)))

        self.safety.set_controls_allowed(1)
        self._set_prev_desired_angle(a)
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        self.assertFalse(self._tx(self._angle_cmd_msg(a - common.sign_of(a) * (max_delta_down + 1.1), True)))

        # Inactive (LKASteerDirection=0): only angle=0 allowed (inactive_angle_is_zero).
        # Carcontroller sends 0 in inactive mode; ECU ignores the angle anyway.
        self.safety.set_controls_allowed(0)
        self.assertEqual(a == 0, self._tx(self._angle_cmd_msg(a, False)))

  @unittest.skip("addr 0x51 (FSM0) is also used by Hyundai CANFD — known whitelist overlap")
  def test_tx_hook_on_wrong_safety_mode(self):
    pass

if __name__ == "__main__":
  unittest.main()
