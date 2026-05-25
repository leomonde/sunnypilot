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


def create_longitudinal(packer, stock_fsm3, accel, acc_check):
  # pass stock FSM3 verbatim except ACC_AccelerationRequest and ACC_Check; bit flip faults ECU (drive 27)
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


# ── Virtual Lead Car (VLC) ────────────────────────────────────────────────────
# Coefficients from linear regression on V60 log data (41 k samples, FrontCar=1):
#   lead_speed_kmh = A_LS * v_ego_ms + B_LS * accel_mss + C_LS  (R²=0.945)
#   distance_m     = A_D  * v_ego_ms + B_D  * accel_mss + C_D   (R²=0.523)
_VLC_A_LS, _VLC_B_LS, _VLC_C_LS = 3.141, 2.667, 3.835
_VLC_A_D,  _VLC_B_D,  _VLC_C_D  = 1.621, 4.002, -1.097

# FSM4 Byte_5: high-nibble encodes braking mode observed in stock data.
# 0xB0 = mild tracking; 0xF0 = strong braking (threshold ~-0.5 m/s²).
# Low nibble 0x3 is the mid-range counter value; actual counter cycles 1-4.
_VLC_BYTE5_MILD   = 0xB3   # 179
_VLC_BYTE5_STRONG = 0xF3   # 243


def _vlc_values(accel: float, v_ego_ms: float) -> tuple[int, int, int, int]:
  """Return (lead_speed_kmh, distance_m, target_state, lead_conf) for a virtual lead car.

  lead_speed_kmh and distance_m are rounded integers ready for packing.
  target_state: 184 = mild braking, 188 = strong braking (bit 2 = active brake flag).
  lead_conf: fixed high-confidence value derived from log statistics.
  """
  lead_speed = int(round(max(0.0, min(250.0, _VLC_A_LS * v_ego_ms + _VLC_B_LS * accel + _VLC_C_LS))))
  distance   = int(round(max(8.0, min(120.0, _VLC_A_D  * v_ego_ms + _VLC_B_D  * accel + _VLC_C_D))))
  target_state = 184 if accel >= -0.5 else 188
  lead_conf    = 240
  return lead_speed, distance, target_state, lead_conf


def create_radar(packer, stock_fsm1, long_active: bool, accel: float = 0.0, v_ego_ms: float = 0.0):
  # When long_active: replace ACC_Distance, ACC_LeadConf, ACC_TargetState with
  # virtual lead car values so the car's ACC accepts the desired deceleration.
  # Keep Byte_3..7 from stock in both modes to avoid triggering unknown checks.
  if long_active:
    _, distance, target_state, lead_conf = _vlc_values(accel, v_ego_ms)
    values = {
      "ACC_Distance":    distance,
      "ACC_LeadConf":    lead_conf,
      "ACC_TargetState": target_state,
    }
  else:
    values = {
      "ACC_Distance":    stock_fsm1["ACC_Distance"],
      "ACC_LeadConf":    stock_fsm1["ACC_LeadConf"],
      "ACC_TargetState": stock_fsm1["ACC_TargetState"],
    }

  values |= {s: stock_fsm1[s] for s in ("Byte_3", "Byte_4", "Byte_5", "Byte_6", "Byte_7")}
  return packer.make_can_msg("FSM1", 0, values)


def create_fsm4(packer, stock_fsm4, long_active: bool, accel: float = 0.0, v_ego_ms: float = 0.0, counter: int = 0):
  # When long_active: build FSM4 with virtual ACC_LeadSpeed and braking-mode Byte_5.
  # Byte_0 toggles 0x55/0xAA each message (observed in stock data).
  # Byte_1 = 0xF1 (241, fixed in stock).
  # Byte_2 and Byte_6 carry rolling counters; use counter mod 16.
  # Byte_4 = 0x8B (139, dominant value in stock data).
  # Byte_7 = 0.
  if long_active:
    lead_speed, _, _, _ = _vlc_values(accel, v_ego_ms)
    byte5 = _VLC_BYTE5_MILD if accel >= -0.5 else _VLC_BYTE5_STRONG
    values = {
      "Byte_0":        0x55 if (counter % 2 == 0) else 0xAA,
      "Byte_1":        0xF1,
      "Byte_2":        counter % 16,
      "ACC_LeadSpeed": lead_speed,
      "Byte_4":        0x8B,
      "Byte_5":        byte5,
      "Byte_6":        counter % 16,
      "Byte_7":        0,
    }
  else:
    values = {s: stock_fsm4[s] for s in (
      "Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed",
      "Byte_4", "Byte_5", "Byte_6", "Byte_7",
    )}
  return packer.make_can_msg("FSM4", 0, values)
