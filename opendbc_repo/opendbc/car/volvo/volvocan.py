def create_button_msg(packer, resume=False, cancel=False, bus=0):
  # TODO: validate
  msg = {
    "ACCOnOffBtn": cancel,
    "ACCOnOffBtnInv": not cancel,
    "ACCResumeBtn": resume,
    "ACCResumeBtnInv": not resume,
  }
  return packer.make_can_msg("CCButtons", bus, msg)


def create_lkas_state_msg(packer, steering_angle: float, stock_values: dict):
  # zero LKATorque/LKAActive to prevent stock LKA interference
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
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03
  trqlim = dat[2]

  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  return s ^ 0xFF


def create_lka_msg(packer, apply_steer: float, steer_direction: int):
  values = {
    "LKAAngleReq": apply_steer,
    "LKASteerDirection": steer_direction,
    "TrqLim": 0,

    "SET_X_22": 0x25,
    "SET_X_02": 0,
    "SET_X_10": 0x10,
    "SET_X_A4": 0xa7,
  }

  # calculate checksum
  dat = packer.make_can_msg("FSM2", 0, values)[1]
  values["Checksum"] = calculate_lka_checksum(dat)

  return packer.make_can_msg("FSM2", 0, values)
