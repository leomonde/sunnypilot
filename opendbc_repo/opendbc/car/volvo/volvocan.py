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
  # ACC_FaultFlag (bit 5 of former byte 7) is forced to 0 — OP must never signal fault itself.
  values = {s: stock_fsm3[s] for s in (
    "ACC_Standstill",
    "Byte_01",
    "Byte_02",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
  )}
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
    "ACC_FaultFlag": 0,
  }
  return packer.make_can_msg("FSM3", 0, values)


# ── Virtual Lead Car (VLC) ────────────────────────────────────────────────────
# Reverse-engineered from V60 stock camera (route 000005ca, 28k lead-active samples).
# Linear regression for kinematic values:
#   lead_speed_kmh = A_LS * v_ego_ms + B_LS * accel_mss + C_LS  (R²=0.945)
#   distance_m     = A_D  * v_ego_ms + B_D  * accel_mss + C_D   (R²=0.523)
_VLC_A_LS, _VLC_B_LS, _VLC_C_LS = 3.141, 2.667, 3.835
_VLC_A_D,  _VLC_B_D,  _VLC_C_D  = 1.621, 4.002, -1.097

# Strong-braking threshold for ACC_TargetState bit-2 (188 vs 184) and FSM4 B5.
# Observed transition point in stock data: 50% strong at accel ≈ -0.32 m/s².
_VLC_STRONG_THRESHOLD = -0.32

# ACC_LeadConf: stock emits 243-255 (255 dominant ~69%). 240 is out of range.
_VLC_LEAD_CONF = 255

# FSM4 Byte_0 cycle: stock runs 9-9-12 frames alternating 0x55/0xAA (period 60).
# Phase boundaries (cumulative): 9, 18, 30, 39, 48, 60
# Compare to old toggle-per-frame which gives 50% match only.
_VLC_B0_BOUNDARIES = (9, 18, 30, 39, 48, 60)


def _vlc_speed_distance(accel: float, v_ego_ms: float) -> tuple[int, int]:
  """Lead speed (km/h) and distance (m) for virtual lead car."""
  lead_speed = int(round(max(0.0, min(250.0, _VLC_A_LS * v_ego_ms + _VLC_B_LS * accel + _VLC_C_LS))))
  distance   = int(round(max(3.0,  min(120.0, _VLC_A_D  * v_ego_ms + _VLC_B_D  * accel + _VLC_C_D))))
  return lead_speed, distance


def _vlc_fsm4_byte0(counter: int) -> int:
  """FSM4 B0: 9-9-12 frame runs alternating 0x55/0xAA (period 60). Matches stock cadence."""
  phase = counter % 60
  for i, boundary in enumerate(_VLC_B0_BOUNDARIES):
    if phase < boundary:
      return 0x55 if (i % 2 == 0) else 0xAA
  return 0x55  # unreachable


def _vlc_fsm4_byte5(accel: float, v_ego_ms: float) -> int:
  """FSM4 B5 (Radar_BrakingMode): stock-observed rule, accel-only.

  Validated against 46k stock samples (real lead 000005ca + stock-long 000005cf seg 1-3,
  vEgo 0-111 km/h). Previous version had a velocity guard that emitted 0xB3 for
  accel=-1.40 at 108 km/h (combination never seen in stock), causing ECM to fault
  the ACC in route 000005d5--c6d70b04fe seg 0 (t=33.355s).

  Stock-observed mapping by accel (any speed up to 111 km/h):
    0xF1  (very strong): accel < -2.0    (stock: -2.88 to -2.08, 246 samples)
    0xF3  (hydraulic):   -2.0 ≤ accel < -0.5  (stock: -1.44 to -0.08, 5840 samples,
                                                covers any vEgo observed)
    0xB3  (light cruise braking): -0.5 ≤ accel < -0.05
    0xB4  (cruise neutral/+):     accel >= -0.05   (DOMINANT in stock)

  Omitted (stock uses rarely, narrow conditions; OP stays within "safe subset"):
    0xF2  (1.7% of stock): use 0xF3 instead — covers same ranges
    0xF4, 0xB5 (~3% combined): edge cases, 0xB3/0xB4 cover the relevant ranges

  v_ego_ms kept in signature for compatibility / future refinements.
  """
  if accel < -2.0:
    return 0xF1
  if accel < -0.5:
    return 0xF3
  if accel < -0.05:
    return 0xB3
  return 0xB4


def create_radar(packer, stock_fsm1, long_active: bool, accel: float = 0.0,
                 v_ego_ms: float = 0.0, strong_braking: bool = False):
  # When long_active: replace ACC_Distance, ACC_LeadConf, ACC_TargetState with
  # virtual lead car values so the car's ACC accepts the desired deceleration.
  # B2 (ACC_TargetState) must follow the stock 5-frame header/data cycle:
  #   data frame (4 of 5): B2 = 184 mild / 188 strong (bit2 = strong flag)
  #   header frame (1 of 5): B2 = 0   mild / 4   strong (bit2 = strong flag)
  # We detect the header frame from stock's ACC_FrameType (=0 in header, =73 in data),
  # avoiding the need to track the camera's internal counter.
  # `strong_braking` comes from CarController with hysteresis to avoid bit-2 flicker.
  # Byte_3..7 are passthrough from stock — preserves the header/data cycle.
  if long_active:
    _, distance = _vlc_speed_distance(accel, v_ego_ms)
    strong_bit = 4 if strong_braking else 0
    # stock_fsm1["ACC_FrameType"] == 0 marks header frame in stock cycle
    is_header = stock_fsm1["ACC_FrameType"] == 0
    base_target = 0 if is_header else 184
    values = {
      "ACC_Distance":    distance,
      "ACC_LeadConf":    _VLC_LEAD_CONF,
      "ACC_TargetState": base_target | strong_bit,
    }
  else:
    values = {
      "ACC_Distance":    stock_fsm1["ACC_Distance"],
      "ACC_LeadConf":    stock_fsm1["ACC_LeadConf"],
      "ACC_TargetState": stock_fsm1["ACC_TargetState"],
    }

  values |= {s: stock_fsm1[s] for s in ("Byte_3", "ACC_FrameType", "Byte_5", "Byte_6", "Byte_7")}
  return packer.make_can_msg("FSM1", 0, values)


def create_fsm4(packer, stock_fsm4, long_active: bool, accel: float = 0.0, v_ego_ms: float = 0.0,
                counter: int = 0, strong_braking: bool = False):
  # FSM4 when long_active: all bytes synthesized from stock-reverse-engineered rules.
  #   Radar_Heartbeat       (B0): 9-9-12 frame runs alternating 0x55/0xAA (was toggle, 50% match)
  #   Radar_StatusFlag      (B1): 0xF9 when standstill + strong braking, else 0xF1
  #   Radar_LeadVelocityAlt (B2): linear regression with LeadSpeed (was counter%16, 2.8% match)
  #   ACC_LeadSpeed         (B3): from regression (unchanged, MAE ~3.4 km/h)
  #   Byte_4                (B4): 0x8B fixed (143 ocasional ignored, 97% match)
  #   Radar_BrakingMode     (B5): speed-aware 7-level scale (see _vlc_fsm4_byte5)
  #   Radar_CRC             (B6): PASSTHROUGH — stock emits 256 unique values; likely
  #                               checksum/rolling-code from internal radar firmware.
  #   Byte_7                (B7): 0 fixed (100% match)
  # `strong_braking` from CarController (with hysteresis) gates the StatusFlag standstill mark.
  if long_active:
    lead_speed, _ = _vlc_speed_distance(accel, v_ego_ms)
    values = {
      "Radar_Heartbeat":       _vlc_fsm4_byte0(counter),
      "Radar_StatusFlag":      0xF9 if (v_ego_ms < 1.0 and strong_braking) else 0xF1,
      "Radar_LeadVelocityAlt": max(0, min(255, int(round(0.977 * lead_speed + 0.752)))),
      "ACC_LeadSpeed":         lead_speed,
      "Byte_4":                0x8B,
      "Radar_BrakingMode":     _vlc_fsm4_byte5(accel, v_ego_ms),
      "Radar_CRC":             stock_fsm4["Radar_CRC"],
      "Byte_7":                0,
    }
  else:
    values = {s: stock_fsm4[s] for s in (
      "Radar_Heartbeat", "Radar_StatusFlag", "Radar_LeadVelocityAlt", "ACC_LeadSpeed",
      "Byte_4", "Radar_BrakingMode", "Radar_CRC", "Byte_7",
    )}
  return packer.make_can_msg("FSM4", 0, values)
