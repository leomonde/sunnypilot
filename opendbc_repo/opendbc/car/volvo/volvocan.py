def create_fsm0(packer, stock_fsm0, front_car_override=None):
  # Pass stock FSM0 through, optionally overriding ACC_FrontCar.
  # Byte_0 (rolling counter) and Byte_7 (checksum) are preserved from stock;
  # analysis of raw frames confirmed byte7 does NOT depend on byte2 signals
  # (ACC_FrontCar/Available/Enabled), so flipping ACC_FrontCar is checksum-safe.
  # Byte_2_hi (bit23 of byte2) is always 1 in observed data — hardcoded.
  values = {s: stock_fsm0[s] for s in (
    "Byte_0", "Byte_1",
    "ACC_Available", "ACC_Enabled",
    "ACC_BrakeAlert",
    "Byte_4", "Byte_5", "Byte_6", "Byte_7",
  )}
  values["Byte_2_hi"] = 1  # bit23 of byte2 — always 1 in stock cam data
  values["ACC_FrontCar"] = int(front_car_override) if front_car_override is not None else int(stock_fsm0["ACC_FrontCar"])
  return packer.make_can_msg("FSM0", 0, values)


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
  # Byte_5 passes through from stock unchanged.
  # Log analysis (drive 54c seg 1) showed stock sends B5=0x02 when ACCELERATING (+1.6 m/s²),
  # not when braking — the previous | 0x02 override was semantically wrong and set it
  # persistently during all negative accel, while stock only sets it momentarily as part
  # of an internal state pattern. Persistent B5=0x02 during braking triggered an ACC fault.
  return packer.make_can_msg("FSM3", 0, values)


def create_fsm4(packer, stock_fsm4, lead_speed_kmh=None):
  # Pass stock FSM4 through, optionally overriding ACC_LeadSpeed with a virtual
  # lead speed (km/h, integer) to create a closing rate that grants ECM braking
  # authority when OP requests deceleration (drive 4e1).
  values = {s: stock_fsm4[s] for s in (
    "Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed",
    "Byte_4", "Byte_5", "Byte_6", "Byte_7",
  )}
  if lead_speed_kmh is not None:
    values["ACC_LeadSpeed"] = max(0, int(round(lead_speed_kmh)))
  return packer.make_can_msg("FSM4", 0, values)


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
  stock_tgt = int(stock_fsm1["ACC_TargetState"])
  # TEST3: only inject virtual lead when stock has NO real lead (tgt=0x00).
  # Log analysis (drive 553 vs 556): when stock already tracks a lead (0xB8/0xBC/0x04),
  # OP injecting B8 at a different distance breaks kinematic consistency and triggers
  # ECM fault. When stock has a real lead, ECM already has hydraulic-brake authority —
  # no injection needed. Virtual lead only needed when stock sees no target at all.
  if virt_dist is not None and stock_tgt == 0x00 and virt_dist < int(stock_fsm1["ACC_Distance"]):
    values["ACC_Distance"] = virt_dist
    values["ACC_LeadConf"] = virt_b1 if virt_b1 is not None else 0xeb
    # TEST4: escalate to 0xBC when dist ≤ 12m — ECM only authorizes hydraulic braking
    # with "confirmed close" state; 0xB8 (tracked) is not enough for braking authority.
    values["ACC_TargetState"] = 0xbc if virt_dist <= 12 else 0xb8
    values["Byte_4"] = 0x49
    values["Byte_6"] = 0x74
  return packer.make_can_msg("FSM1", 0, values)
