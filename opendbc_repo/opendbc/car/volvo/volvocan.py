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
# Simplified strategy: only inject VLC when OP commands hydraulic braking
# (accel < -0.5, where stock starts using B5 high-nibble F).
#
# Without VLC (cruise / acceleration / engine braking): pretend no lead.
#   ACC_Distance=255, ACC_LeadConf=0, ACC_TargetState=mild (184/0), B5=B3/B4.
#   AccelRequest still commanded by OP; ECM accepts mild commands without lead.
#
# With VLC (hydraulic braking): inject close + slower lead to justify the brake.
#   ACC_Distance/LeadSpeed from formulas (validated against stock observed values).
#   ACC_LeadConf=255, ACC_TargetState=strong (188/4), B5=F3/F2/F1.
#
# Hysteresis on activation prevents VLC flicker around the -0.5 boundary:
#   enter VLC when accel < -0.55
#   exit  VLC when accel > -0.40
#
# Transição abrupta on VLC enter/exit is accepted (mimics a lead car cutting
# into / out of lane).
#
# Distance / LeadSpeed formulas (option "C" hybrid simples-com-accel):
#   distance_m   = max(8, v_ego_ms * 1.5 + accel * 4)
#   lead_kmh     = max(0, v_ego_kmh - 5 - (-accel) * 8)
#
# Validation against 2.3k stock samples (accel<-0.5, lead present):
#   distance MAE ≈ 3m   (stock median 10m, vEgo median 14 km/h)
#   lead_kmh MAE ≈ 6 km/h

_VLC_LEAD_CONF = 255       # stock emits 243-255 with lead (255 dominant ~69%)
_VLC_ENTER_THRESHOLD = -0.55  # accel must drop below this to enter VLC mode
_VLC_EXIT_THRESHOLD  = -0.40  # accel must rise above this to exit VLC mode


def _vlc_brake_values(accel: float, v_ego_ms: float) -> tuple[int, int]:
  """Distance (m) and lead speed (km/h) for the virtual lead during hydraulic braking.
  Formula 'C': headway-based with accel adjustment, gap proportional to accel intensity.
  """
  v_ego_kmh = v_ego_ms * 3.6
  distance_m = max(8.0, v_ego_ms * 1.5 + accel * 4.0)
  lead_kmh   = max(0.0, v_ego_kmh - 5.0 - (-accel) * 8.0)
  return int(round(distance_m)), int(round(lead_kmh))


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


def create_radar(packer, stock_fsm1, long_active: bool, vlc_active: bool = False,
                 accel: float = 0.0, v_ego_ms: float = 0.0,
                 strong_braking: bool = False):
  # `long_active`: OP is controlling longitudinal (= injecting on FSM1/FSM3/FSM4).
  # `vlc_active`: VLC mode (hydraulic braking) — inject close lead. Else: pretend no lead.
  #
  # B2 (ACC_TargetState) follows the stock 5-frame header/data cycle:
  #   data frame (4 of 5): 184 mild / 188 strong (bit2 = strong flag)
  #   header frame (1 of 5): 0    mild / 4   strong (bit2 = strong flag)
  # Header detected from stock's ACC_FrameType (=0 in header, =73 in data).
  # B3..B7 are passthrough from stock — preserves the header/data cycle for B4/B6.
  if long_active:
    strong_bit = 4 if strong_braking else 0
    is_header = stock_fsm1["ACC_FrameType"] == 0
    base_target = 0 if is_header else 184
    if vlc_active:
      # Hydraulic braking: inject close lead to justify the brake authority
      distance, _ = _vlc_brake_values(accel, v_ego_ms)
      values = {
        "ACC_Distance":    distance,
        "ACC_LeadConf":    _VLC_LEAD_CONF,
        "ACC_TargetState": base_target | strong_bit,
      }
    else:
      # No VLC: pretend there is no lead car — cruise mode
      values = {
        "ACC_Distance":    255,
        "ACC_LeadConf":    0,
        "ACC_TargetState": base_target,    # always mild (no strong bit)
      }
  else:
    # oplong off — passthrough total
    values = {
      "ACC_Distance":    stock_fsm1["ACC_Distance"],
      "ACC_LeadConf":    stock_fsm1["ACC_LeadConf"],
      "ACC_TargetState": stock_fsm1["ACC_TargetState"],
    }

  values |= {s: stock_fsm1[s] for s in ("Byte_3", "ACC_FrameType", "Byte_5", "Byte_6", "Byte_7")}
  return packer.make_can_msg("FSM1", 0, values)


def create_fsm4(packer, stock_fsm4, long_active: bool, vlc_active: bool = False,
                accel: float = 0.0, v_ego_ms: float = 0.0,
                strong_braking: bool = False):
  # `long_active`: OP controlling longitudinal. `vlc_active`: hydraulic brake mode.
  #
  # Bytes:
  #   Radar_Heartbeat       (B0): PASSTHROUGH from stock — ECM checks cadence
  #   Radar_StatusFlag      (B1): 0xF9 when standstill + strong braking, else 0xF1
  #   Radar_LeadVelocityAlt (B2): linear from lead_kmh (regression with B3)
  #   ACC_LeadSpeed         (B3): VLC lead speed when vlc_active, else 0
  #   Byte_4                (B4): 0x8B fixed
  #   Radar_BrakingMode     (B5): accel-only scale (_vlc_fsm4_byte5)
  #   Radar_CRC             (B6): PASSTHROUGH — checksum from radar firmware
  #   Byte_7                (B7): 0 fixed
  if long_active:
    if vlc_active:
      _, lead_kmh = _vlc_brake_values(accel, v_ego_ms)
    else:
      lead_kmh = 0    # no lead → lead speed irrelevant
    values = {
      "Radar_Heartbeat":       stock_fsm4["Radar_Heartbeat"],
      "Radar_StatusFlag":      0xF9 if (v_ego_ms < 1.0 and strong_braking) else 0xF1,
      "Radar_LeadVelocityAlt": max(0, min(255, int(round(0.977 * lead_kmh + 0.752)))),
      "ACC_LeadSpeed":         lead_kmh,
      "Byte_4":                0x8B,
      "Radar_BrakingMode":     _vlc_fsm4_byte5(accel, v_ego_ms),
      "Radar_CRC":             stock_fsm4["Radar_CRC"],
      "Byte_7":                0,
    }
  else:
    # oplong off — passthrough total
    values = {s: stock_fsm4[s] for s in (
      "Radar_Heartbeat", "Radar_StatusFlag", "Radar_LeadVelocityAlt", "ACC_LeadSpeed",
      "Byte_4", "Radar_BrakingMode", "Radar_CRC", "Byte_7",
    )}
  return packer.make_can_msg("FSM4", 0, values)
