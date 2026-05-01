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


def create_longitudinal(packer, stock_fsm3, accel, acc_check, acc_standstill=None):
  # pass stock FSM3 verbatim except ACC_AccelerationRequest, ACC_Check and (optionally) ACC_Standstill
  # bit flip faults ECU (drive 27); acc_standstill=None means pass stock through
  values = {s: stock_fsm3[s] for s in (
    "Byte_01",
    "Byte_02",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  values["ACC_Standstill"] = int(acc_standstill) if acc_standstill is not None else int(stock_fsm3["ACC_Standstill"])
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  return packer.make_can_msg("FSM3", 0, values)


def create_radar(packer, stock_fsm1, long_active, virt_dist=None, virt_b1=None):
  # Pass stock FSM1 through, optionally overriding ACC_Distance/ACC_LeadConf/ACC_TargetState
  # with a virtual lead to grant the ECU hydraulic-brake authority (drive 4d4).
  # Virtual values are only applied when they are closer than the stock distance,
  # so a real lead is never hidden from the ECU.
  _ = long_active  # kept for signature stability
  values = {s: stock_fsm1[s] for s in (
    "ACC_Distance",
    "ACC_LeadConf",
    "ACC_TargetState",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  if virt_dist is not None and virt_dist < int(stock_fsm1["ACC_Distance"]):
    values["ACC_Distance"] = virt_dist
    values["ACC_LeadConf"] = virt_b1 if virt_b1 is not None else 0xeb
    if values["ACC_TargetState"] == 0:
      values["ACC_TargetState"] = 0xb8  # "tracked target"; avoids inconsistent FSM1 payload
  return packer.make_can_msg("FSM1", 0, values)
