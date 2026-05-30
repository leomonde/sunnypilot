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


def create_longitudinal(packer, stock_fsm3, accel, acc_check, braking: bool = False,
                        has_real_lead: bool = False, emergency_brake: bool = False):
  # Passthrough stock FSM3, override AccelRequest/ACC_Check, force ACC_FaultFlag=0.
  # bit 6 of byte 0 = "ACC commanding hydraulic brake".
  #   - no lead: OP decides based on vlc_active (braking flag)
  #   - with lead: passthrough stock to stay in its brake mode
  #   - emergency: force bit 6 ON to engage hydraulic actuator regardless of stock
  byte_01 = int(stock_fsm3["Byte_01"])
  if (braking and not has_real_lead) or emergency_brake:
    byte_01 |= 0b01000   # bit 6 of byte 0 (inside Byte_01 signal which covers bits 7-3)

  values = {
    "ACC_Standstill": stock_fsm3["ACC_Standstill"],
    "Byte_01":        byte_01,
    "Byte_02":        stock_fsm3["Byte_02"],
    "Byte_2":         stock_fsm3["Byte_2"],
    "Byte_3":         stock_fsm3["Byte_3"],
    "Byte_4":         stock_fsm3["Byte_4"],
    "Byte_5":         stock_fsm3["Byte_5"],
    "Byte_6":         stock_fsm3["Byte_6"],
    "ACC_AccelerationRequest": accel,
    "ACC_Check":      acc_check,
    "ACC_FaultFlag":  0,
  }
  return packer.make_can_msg("FSM3", 0, values)


# ── Stock Cruise Reduction (no VLC) ──────────────────────────────────────────
# Stock freia até -2.0 m/s² sem lead (Distance=255). OP imita esse padrão em vez de
# inventar lead virtual. Ref: 000005e1 log 1 (no-lead braking) e logs 5-6 (with lead).

_VLC_LEAD_CONF = 255          # stock emits 243-255 (255 dominant ~69%)
_VLC_ENTER_THRESHOLD = -0.55  # accel < enter → braking mode (hysteresis)
_VLC_EXIT_THRESHOLD  = -0.40


def _vlc_fsm4_byte5(accel: float, v_ego_ms: float) -> int:
  # FSM4 B5 (Radar_BrakingMode) by accel — stock observed mapping (000005e1 log 1):
  # 0xF1 strong (<-2), 0xF2 moderate, 0xF3 light hydraulic, 0xB3 cruise brake, 0xB4 neutral+.
  if accel < -2.0:
    return 0xF1
  if accel < -1.0:
    return 0xF2
  if accel < -0.5:
    return 0xF3
  if accel < -0.05:
    return 0xB3
  return 0xB4


def create_radar(packer, stock_fsm1, long_active: bool, strong_braking: bool = False,
                 has_real_lead: bool = False, emergency_brake: bool = False):
  # FSM1 modes:
  # - long_active + no lead: Phase 1 — Distance=255/LeadConf=255 (stock cruise reduction).
  # - long_active + lead:    Phase 2 — passthrough lead data; OP only adds TargetState bit 2.
  # - !long_active:          passthrough stock.
  # Emergency: with lead, force TargetState bit 2 even if stock isn't asserting it.
  if long_active and has_real_lead:
    stock_target = int(stock_fsm1["ACC_TargetState"])
    # Only amplify strong-brake bit when stock also has it set — unless emergency,
    # in which case OP forces bit 2 to keep all flags consistent for hydraulic braking.
    stock_has_strong = bool(stock_target & 0b100)
    add_bit2 = (strong_braking and stock_has_strong) or emergency_brake
    values = {
      "ACC_Distance":    stock_fsm1["ACC_Distance"],
      "ACC_LeadConf":    stock_fsm1["ACC_LeadConf"],
      "ACC_TargetState": stock_target | (0b100 if add_bit2 else 0),
    }
  elif long_active:
    strong_bit = 4 if strong_braking else 0
    is_header = stock_fsm1["ACC_FrameType"] == 0
    base_target = 0 if is_header else 184
    values = {
      "ACC_Distance":    255,
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


def create_fsm4(packer, stock_fsm4, long_active: bool, accel: float = 0.0,
                v_ego_ms: float = 0.0, strong_braking: bool = False,
                has_real_lead: bool = False, emergency_brake: bool = False):
  # FSM4 modes (Heartbeat/CRC always passthrough — ECM checks cadence):
  # - long_active + lead:   Phase 2 — passthrough lead fields; OP overrides BrakingMode.
  # - long_active + nolead: Phase 1 — synthetic no-lead pattern (StatusFlag/B4/LeadSpeed=0).
  # - !long_active:         full passthrough.
  # Emergency: with lead, override BrakingMode to engage hydraulic (F* range) when
  # OP needs strong brake and stock is still in engine-brake mode.
  if long_active and has_real_lead:
    # With lead: BrakingMode also passthrough — stay in stock's brake actuator mode
    # (cruise vs hydraulic). OP only modulates AccelRequest magnitude within clamp.
    values = {s: stock_fsm4[s] for s in (
      "Radar_Heartbeat", "Radar_StatusFlag", "Radar_LeadVelocityAlt", "ACC_LeadSpeed",
      "Byte_4", "Radar_BrakingMode", "Radar_CRC", "Byte_7",
    )}
    if emergency_brake:
      # Force hydraulic mode: ECM caliper engages so car can deliver OP's accel.
      values["Radar_BrakingMode"] = _vlc_fsm4_byte5(accel, v_ego_ms)
  elif long_active:
    values = {
      "Radar_Heartbeat":       stock_fsm4["Radar_Heartbeat"],
      "Radar_StatusFlag":      0xF9 if (v_ego_ms < 1.0 and strong_braking) else 0xF1,
      "Radar_LeadVelocityAlt": 0,
      "ACC_LeadSpeed":         0,
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
