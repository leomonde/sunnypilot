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


def create_fsm0(packer, stock_fsm0, virtual_lead_active=False):
  # Relay FSM0 at 100 Hz; override ACC_FrontCar=1 when virtual lead is active
  # so the ECU enters follow mode (not just cruise). Without this bit set, the
  # ECU ignores FSM1 distance and FSM4 lead speed entirely (observed: route 599).
  # All non-overridden bits are forwarded verbatim using the raw byte fields
  # added to the DBC (Byte_0/1, Byte_2_upper, Byte_3_high/low, Byte_4-7).
  values = {
    "ACC_Available":  int(stock_fsm0["ACC_Available"]),
    "ACC_Enabled":    int(stock_fsm0["ACC_Enabled"]),
    "ACC_BrakeAlert": int(stock_fsm0["ACC_BrakeAlert"]),
    "ACC_FrontCar":   1 if virtual_lead_active else int(stock_fsm0["ACC_FrontCar"]),
    "Byte_0":         int(stock_fsm0["Byte_0"]),
    "Byte_1":         int(stock_fsm0["Byte_1"]),
    "Byte_2_upper":   int(stock_fsm0["Byte_2_upper"]),
    "Byte_3_high":    int(stock_fsm0["Byte_3_high"]),
    "Byte_3_low":     int(stock_fsm0["Byte_3_low"]),
    "Byte_4":         int(stock_fsm0["Byte_4"]),
    "Byte_5":         int(stock_fsm0["Byte_5"]),
    "Byte_6":         int(stock_fsm0["Byte_6"]),
    "Byte_7":         int(stock_fsm0["Byte_7"]),
  }
  return packer.make_can_msg("FSM0", 0, values)


def create_longitudinal(packer, stock_fsm3, accel, acc_check, byte2=None, byte01=None):
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
    # Byte_01 bit 3 (value 8) signals "FrontCar active" to the ECU.
    # Stock FSM3 sends 29 (0b11101, <55 km/h) or 21 (0b10101, >63 km/h) with lead car.
    # Caller passes the speed-dependent value with 7 km/h hysteresis (route 5a0 analysis).
    values["Byte_01"] = byte01 if byte01 is not None else 29
    values["Byte_02"] = 1
    # Route 5a0: ACC_Standstill must be 0 when virtual lead is active.
    # If left at 1 (stock transient) the ECU ignores brake requests entirely.
    values["ACC_Standstill"] = 0
  return packer.make_can_msg("FSM3", 0, values)


def compute_virtual_lead(accel_ms2: float, vEgo_ms: float) -> dict:
  """
  Invert the empirical Volvo ACC model to obtain (dist_virtual, vLead_virtual)
  that causes the ACC to produce accel_ms2.

  Model (45 732 samples, RMSE 0.144 m/s²):
    AccelReq = KP*(dist - TGAP*vEgo) + KV*ΔV_kmh + OFFSET

  Strategy: solve for dist_v with ΔV=0 first. D_MIN/D_MAX are hardware
  limits — only then switch to ΔV to cover what distance alone cannot.
  This makes the virtual distance scale naturally with speed.
  """
  KP     = 0.01893   # m/s² per m of distance error
  KV     = 0.06595   # m/s² per km/h of ΔV
  OFFSET = -0.1049   # m/s²
  TGAP   = 0.8       # seconds
  D_MIN  = 15.0      # m (FSM hardware minimum observed)
  D_MAX  = 70.0      # m (FSM hardware maximum observed)

  vEgo_kmh    = vEgo_ms * 3.6
  dist_target = TGAP * vEgo_ms
  accel       = max(-1.36, min(1.36, accel_ms2))

  # Primary inversion: keep ΔV = 0, vary distance
  delta_v = 0.0
  dist_v  = (accel - OFFSET) / KP + dist_target

  if dist_v > D_MAX:
    # Too far — cap at D_MAX and compensate with positive ΔV (lead faster)
    dist_v  = D_MAX
    delta_v = (accel - KP * (D_MAX - dist_target) - OFFSET) / KV
  elif dist_v < D_MIN:
    # Too close — cap at D_MIN and compensate with negative ΔV (lead slower)
    dist_v  = D_MIN
    delta_v = (accel - KP * (D_MIN - dist_target) - OFFSET) / KV

  vLead_kmh = max(0.0, vEgo_kmh + delta_v)
  delta_v   = vLead_kmh - vEgo_kmh  # recalculate after 0-clamp

  # FSM3 state hints based on acceleration direction and magnitude
  if accel >= 0:
    target_state = 0xB8
    byte2_fsm3   = 214
  else:
    # 0xBC activates stronger braking path; real condition (route 5a0): delta_v < -2.5 km/h AND dist < 30 m
    target_state = 0xBC if (delta_v < -2.5 and dist_v < 30.0) else 0xB8
    byte2_fsm3   = 212 if accel < -0.1 else 214

  # confidence 255 close-in, decays ~0.15/m beyond 20 m, floor 248
  # Route 58f showed ACC_LeadConf ≥ 248 consistently; lower values caused FSM
  # to reject the target. Keep floor at 248 to match real radar behaviour.
  lead_conf = max(248, min(255, round(255 - max(0.0, dist_v - 20.0) * 0.15)))

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
    target_state = virtual_lead['target_state']
    # Route 58f analysis: Byte_4=73 and Byte_6=116 when TargetState=184/188
    # (active follow), else 0/20 (no target). Must derive from target_state,
    # NOT pass stock values — stock had no virtual lead so its Byte_4/6 were 0.
    following = target_state in (0xB8, 0xBC)  # 184 steady-follow, 188 active-tracking
    values = {
      "ACC_Distance":    int(max(15, min(70, virtual_lead['dist_virtual']))),
      "ACC_LeadConf":    virtual_lead['lead_conf'],
      "ACC_TargetState": target_state,
      "Byte_3": 0,
      "Byte_4": 73 if following else 0,
      "Byte_5": 227,
      "Byte_6": 116 if following else 20,
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


def create_lead_speed(packer, vLead_kmh: float, stock_fsm4: dict, byte0: int = 85):
  # Byte_0 alternates 85/170 every FSM4 send — rolling 1-bit counter (route 5a0 analysis).
  # Caller is responsible for toggling byte0 on each call (see carcontroller _fsm4_toggle).
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


def create_fsm4_passthrough(packer, stock_fsm4: dict):
  # Pure relay of stock FSM4 — used when oplong is inactive so the ECU receives
  # exactly what the camera would send. create_lead_speed has hardcoded Byte_1/
  # Byte_4 tuned for virtual-lead mode; those values are wrong in normal ACC mode
  # and cause ECU consistency faults when ICBM is active with oplong disabled.
  values = {s: stock_fsm4[s] for s in (
    "Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed",
    "Byte_4", "Byte_5", "Byte_6", "Byte_7",
  )}
  return packer.make_can_msg("FSM4", 0, values)
