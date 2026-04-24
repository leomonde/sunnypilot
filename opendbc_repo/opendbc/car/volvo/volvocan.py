def create_button_msg(packer, resume=False, cancel=False, set_plus=False, minus=False, bus=0):
  # TODO: validate
  msg = {
    "ACCOnOffBtn": cancel,
    "ACCOnOffBtnInv": not cancel,
    "ACCResumeBtn": resume,
    "ACCResumeBtnInv": not resume,
    "ACCSetBtn": set_plus,
    "ACCSetBtnInv": not set_plus,
    "ACCMinusBtn": minus,
    "ACCMinusBtnInv": not minus,
  }
  return packer.make_can_msg("CCButtons", bus, msg)


def create_acc_state_msg(packer):
  msg = {
    "ACC_Check": 1,
  }
  return packer.make_can_msg("FSM3", 0, msg)


def create_lkas_state_msg(packer, steering_angle: float, stock_values: dict):
  # Manipulate data from servo to FSM
  # Set LKATorque and LKAActive to zero otherwise LKA will be disabled. (Check dbc)
  msg = {
    "LKATorque": 0,
    "SteeringAngleServo": steering_angle,
    "byte0": stock_values["byte0"],
    "byte4": stock_values["byte4"],
    "byte7": stock_values["byte7"],
    "LKAActive": int(stock_values["LKAActive"]) & 0xF5,
    "EPSTorque": stock_values["EPSTorque"],
  }
  return packer.make_can_msg("PSCM1", 2, msg)


def calculate_lka_checksum(dat: bytearray) -> int:
  # Input: dat byte array, and fingerprint
  # Steering direction = 0 -> 3
  # TrqLim = 0 -> 255
  # Steering angle request = -360 -> 360

  # Extract LKAAngleRequest, LKADirection and Unknown
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03
  trqlim = dat[2]

  # Sum of all bytes, carry ignored.
  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF


def create_lka_msg(packer, apply_steer: float, steer_direction: int):
  values = {
    "LKAAngleReq": apply_steer,
    "LKASteerDirection": steer_direction,
    "TrqLim": 0,

    # car specific parameters
    "SET_X_22": 0x25, # Test these values: 0x24, 0x22
    "SET_X_02": 0,    # Test 0x00, 0x02
    "SET_X_10": 0x10, # Test 0x10, 0x1c, 0x18, 0x00
    "SET_X_A4": 0xa7, # Test 0xa4, 0xa6, 0xa5, 0xe5, 0xe7
  }

  # calculate checksum
  dat = packer.make_can_msg("FSM2", 0, values)[1]
  values["Checksum"] = calculate_lka_checksum(dat)

  return packer.make_can_msg("FSM2", 0, values)


def create_longitudinal(packer, stock_fsm3, accel, acc_check):
  # Pass through ALL stock FSM3 bits verbatim so OP's message is byte-identical
  # to stock's latest except for ACC_AccelerationRequest (byte 1) and ACC_Check.
  # This preserves the car's 5-frame validation pattern and all counter/mode
  # bits the ECM checks. Missing any field here will flip a bit in the output
  # and the ECM may fault after accumulated errors (observed in drive 27 at ~30s).
  values = {s: stock_fsm3[s] for s in (
    "ACC_Standstill",
    "Byte_01",
    "Byte_02",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  return packer.make_can_msg("FSM3", 0, values)


def create_radar(packer, stock_fsm1, long_active):
  # Pass through ALL stock FSM1 bytes verbatim, INCLUDING ACC_Distance.
  #
  # Rationale for not spoofing ACC_Distance=255:
  # - OP's planner has radarUnavailable=True → OP doesn't use ACC_Distance anyway
  # - The car's stock ACC uses ACC_Distance to track lead vehicles and decide
  #   whether to let the car slow below its 30 km/h engagement floor. When we
  #   spoofed 255 ("no lead"), stock ACC would disengage when OP commanded
  #   decel below 30, which via pcmCruise also disengaged OP and left the
  #   driver to brake manually.
  # - Letting stock's real ACC_Distance reach the car's ACC means the car
  #   will follow a real lead vehicle to 0 km/h while OP controls the accel
  #   byte — enabling experimental-mode stop-at-light when a lead is present.
  _ = long_active  # kept for signature stability
  values = {s: stock_fsm1[s] for s in (
    "ACC_Distance",
    "Byte_1",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  return packer.make_can_msg("FSM1", 0, values)
