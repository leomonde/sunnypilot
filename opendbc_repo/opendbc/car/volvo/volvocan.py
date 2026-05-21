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


def create_longitudinal(packer, stock_fsm3, accel, acc_check, byte2=None):
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
  if byte2 is not None:
    values["Byte_2"] = byte2
  return packer.make_can_msg("FSM3", 0, values)


def compute_virtual_lead(accel_ms2: float, vEgo_ms: float) -> dict:
  """
  Invert the empirical Volvo ACC model to obtain (dist_virtual, vLead_virtual)
  that causes the ACC to produce accel_ms2.

  Model (45 732 samples, RMSE 0.144 m/s²):
    AccelReq = KP*(dist - TGAP*vEgo) + KV*ΔV_kmh + OFFSET
  """
  KP     = 0.01893   # m/s² per m of distance error
  KV     = 0.06595   # m/s² per km/h of ΔV
  OFFSET = -0.1049   # m/s²
  TGAP   = 0.8       # seconds
  D_MIN  = 15.0      # m (FSM hardware minimum)
  D_MAX  = 70.0      # m (FSM hardware maximum observed)

  vEgo_kmh    = vEgo_ms * 3.6
  dist_target = TGAP * vEgo_ms
  accel       = max(-1.36, min(1.36, accel_ms2))

  if accel >= 0:
    # Acceleration: increase virtual distance, keep ΔV = 0
    delta_v = 0.0
    dist_v  = (accel - OFFSET) / KP + dist_target
    if dist_v > D_MAX:
      dist_v  = D_MAX
      delta_v = (accel - KP * (D_MAX - dist_target) - OFFSET) / KV
    target_state = 0xB8
    byte2_fsm3   = 214
  else:
    # Deceleration: pin distance to minimum, reduce virtual lead speed
    dist_v  = D_MIN
    delta_v = (accel - KP * (D_MIN - dist_target) - OFFSET) / KV
    vLead_kmh = max(0.0, vEgo_kmh + delta_v)
    delta_v   = vLead_kmh - vEgo_kmh
    target_state = 0xBC if accel < -0.2 else 0xB8
    byte2_fsm3   = 212

  vLead_kmh = vEgo_kmh + delta_v
  # confidence 255 close-in, decays ~0.15/m beyond 20 m, floor 241
  lead_conf = max(241, min(255, round(255 - max(0.0, dist_v - 20.0) * 0.15)))

  return {
    'dist_virtual': round(dist_v, 1),
    'vLead_kmh':    round(vLead_kmh, 1),
    'delta_v_kmh':  round(delta_v, 2),
    'target_state': target_state,
    'lead_conf':    lead_conf,
    'byte2_fsm3':   byte2_fsm3,
    'accel_request': accel,
  }


def create_radar(packer, stock_fsm1, virtual_lead=None):
  if virtual_lead is not None:
    values = {
      "ACC_Distance":    int(max(15, min(70, virtual_lead['dist_virtual']))),
      "ACC_LeadConf":    virtual_lead['lead_conf'],
      "ACC_TargetState": virtual_lead['target_state'],
      "Byte_3": 0,
      "Byte_4": int(stock_fsm1.get("Byte_4", 0)),
      "Byte_5": 227,
      "Byte_6": int(stock_fsm1.get("Byte_6", 0)),
      "Byte_7": 8,
    }
  else:
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
  return packer.make_can_msg("FSM1", 0, values)


def create_lead_speed(packer, vLead_kmh: float, stock_fsm4: dict, frame: int):
  # Byte_0 alternates 85/170 — appears to be a rolling bit; match observed pattern
  byte0 = 170 if (frame // 10) % 2 else 85
  values = {
    "Byte_0":       byte0,
    "Byte_1":       241,
    "Byte_2":       int(stock_fsm4.get("Byte_2", 0)),
    "ACC_LeadSpeed": int(max(0, min(255, round(vLead_kmh)))),
    "Byte_4":       139,
    "Byte_5":       int(stock_fsm4.get("Byte_5", 0)),
    "Byte_6":       int(stock_fsm4.get("Byte_6", 0)),
    "Byte_7":       int(stock_fsm4.get("Byte_7", 0)),
  }
  return packer.make_can_msg("FSM4", 0, values)
