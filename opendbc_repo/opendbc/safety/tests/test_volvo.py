#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoSafety(common.CarSafetyTest):

  TX_MSGS = [[0x127, 0], [0x246, 2], [0x051, 0], [0x262, 0], [0x260, 0], [0x270, 0], [0x31A, 0]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: [0x051, 0x262, 0x31A], 2: [0x246]}
  FWD_BLACKLISTED_ADDRS = {2: [0x051, 0x262, 0x31A], 0: [0x246]}

  VOLVO_MAIN_BUS = 0
  VOLVO_CAM_BUS = 2

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, 0)
    self.safety.init_tests()

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

  def _lka_msg(self, enabled: bool, angle: float = 0.0):
    values = {"LKAAngleReq": angle, "LKASteerDirection": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("FSM2", self.VOLVO_MAIN_BUS, values)

  def test_lka_allowed_only_when_controls_allowed(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._lka_msg(enabled=True, angle=5.0)))

  def test_lka_inactive_always_allowed(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(self._lka_msg(enabled=False, angle=0.0)))

  @unittest.skip("addr 0x51 (FSM0) is also used by Hyundai CANFD — known whitelist overlap")
  def test_tx_hook_on_wrong_safety_mode(self):
    pass


if __name__ == "__main__":
  unittest.main()
