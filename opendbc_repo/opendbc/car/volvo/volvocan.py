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
# Stateful lead simulation with physically-plausible motion.
# The ECM authorizes hydraulic braking when it "sees" a close lead car. The OP
# manipulates lead distance + speed to justify the AccelRequest it commands.
# Movement is rate-limited so distance/speed evolve plausibly between frames,
# avoiding the "lead teleporting" pattern that caused ECM faults in earlier
# attempts (e.g. 000005d5 seg 0: VLC distance jumped from 255→17m on activation).
#
# Targets per accel bucket — derived from stock observation (34k samples with
# real lead, 000005ca + 000005cf):
#
#   accel range          headway   min_dist  gap_kmh (lead slower than ego)
#   ≥ 0      (cruise)    cruise:   dist=80m, lead = ego + 2 km/h (afastando levemente)
#   -0.3..0  (very mild) 2.0 s     20 m      3
#   -0.5..-0.3 (mild)    1.5 s     12 m      6
#   -1.0..-0.5 (moderate) 1.2 s    10 m      10
#   -1.5..-1.0 (strong)  1.0 s      8 m      15
#   < -1.5   (very strong) 0.8 s   12 m      25

_VLC_LEAD_CONF = 255          # stock emits 243-255 (255 dominant ~69%)
_VLC_CRUISE_DIST = 80.0       # cruise: lead distante (stock p90=84m)
_VLC_CRUISE_LEAD_OFFSET = 2.0 # km/h above ego in cruise (slight opening)

# Physical motion limits (per second)
_VLC_MAX_CLOSE_RATE = 20.0   # m/s — lead approaching (= 72 km/h relative)
_VLC_MAX_OPEN_RATE  = 10.0   # m/s — lead moving away  (= 36 km/h relative)
_VLC_MAX_LEAD_ACCEL_KMH = 10.8  # km/h per second (= 3 m/s²)


class VLCState:
  """Stateful virtual lead car. Updated once per FSM4 TX (33Hz)."""
  def __init__(self):
    self.distance = _VLC_CRUISE_DIST     # m, init: distant (cruise)
    self.lead_kmh = 0.0                  # km/h, will sync to ego on first update
    self.initialized = False

  def reset(self):
    """Call when entering VLC mode after being inactive."""
    self.distance = _VLC_CRUISE_DIST
    self.initialized = False

  def update(self, accel: float, v_ego_ms: float, dt: float = 0.030):
    """Advance state one step; returns (distance_int, lead_speed_kmh_int)."""
    v_ego_kmh = v_ego_ms * 3.6
    if not self.initialized:
      self.lead_kmh = v_ego_kmh + _VLC_CRUISE_LEAD_OFFSET
      self.initialized = True

    # Target based on accel bucket (stock-observed values)
    if accel >= 0:
      target_dist = _VLC_CRUISE_DIST
      target_lead = v_ego_kmh + _VLC_CRUISE_LEAD_OFFSET
    else:
      if accel >= -0.3:
        headway, min_d, gap = 2.0, 20.0, 3.0
      elif accel >= -0.5:
        headway, min_d, gap = 1.5, 12.0, 6.0
      elif accel >= -1.0:
        headway, min_d, gap = 1.2, 10.0, 10.0
      elif accel >= -1.5:
        headway, min_d, gap = 1.0, 8.0, 15.0
      else:
        headway, min_d, gap = 0.8, 12.0, 25.0
      target_dist = max(min_d, v_ego_ms * headway)
      target_lead = max(0.0, v_ego_kmh - gap)

    # Move distance toward target, rate-limited by physical closing/opening rates
    delta_d = target_dist - self.distance
    delta_d = max(-_VLC_MAX_CLOSE_RATE * dt, min(_VLC_MAX_OPEN_RATE * dt, delta_d))
    self.distance += delta_d

    # Move lead speed toward target, rate-limited by physical accel
    delta_l = target_lead - self.lead_kmh
    delta_l = max(-_VLC_MAX_LEAD_ACCEL_KMH * dt, min(_VLC_MAX_LEAD_ACCEL_KMH * dt, delta_l))
    self.lead_kmh += delta_l

    # Pack into int valid range
    distance_int = max(0, min(255, int(round(self.distance))))
    lead_kmh_int = max(0, min(255, int(round(self.lead_kmh))))
    return distance_int, lead_kmh_int


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


def create_radar(packer, stock_fsm1, long_active: bool, vlc_distance: int = 0,
                 strong_braking: bool = False):
  # When long_active: inject VLC values for ACC_Distance + ACC_LeadConf + ACC_TargetState.
  # `vlc_distance` is computed by VLCState (CarController) and shared between FSM1 + FSM4.
  # B2 (ACC_TargetState) must follow the stock 5-frame header/data cycle:
  #   data frame (4 of 5): B2 = 184 mild / 188 strong (bit2 = strong flag)
  #   header frame (1 of 5): B2 = 0   mild / 4   strong (bit2 = strong flag)
  # We detect the header frame from stock's ACC_FrameType (=0 in header, =73 in data).
  # `strong_braking` comes from CarController (with hysteresis on accel).
  # Byte_3..7 are passthrough from stock — preserves the header/data cycle for B4/B6.
  if long_active:
    strong_bit = 4 if strong_braking else 0
    is_header = stock_fsm1["ACC_FrameType"] == 0
    base_target = 0 if is_header else 184
    values = {
      "ACC_Distance":    vlc_distance,
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
                v_ego_ms: float = 0.0, vlc_lead_kmh: int = 0,
                strong_braking: bool = False):
  # FSM4 when long_active: all bytes synthesized from stock-reverse-engineered rules.
  #   Radar_Heartbeat       (B0): PASSTHROUGH from stock — ECM checks cadence
  #   Radar_StatusFlag      (B1): 0xF9 when standstill + strong braking, else 0xF1
  #   Radar_LeadVelocityAlt (B2): linear from lead_kmh (regression with B3)
  #   ACC_LeadSpeed         (B3): from VLCState (stateful, rate-limited)
  #   Byte_4                (B4): 0x8B fixed (143 ocasional ignored, 97% match)
  #   Radar_BrakingMode     (B5): accel-only scale (see _vlc_fsm4_byte5)
  #   Radar_CRC             (B6): PASSTHROUGH — checksum/rolling-code from radar firmware
  #   Byte_7                (B7): 0 fixed
  if long_active:
    values = {
      "Radar_Heartbeat":       stock_fsm4["Radar_Heartbeat"],
      "Radar_StatusFlag":      0xF9 if (v_ego_ms < 1.0 and strong_braking) else 0xF1,
      "Radar_LeadVelocityAlt": max(0, min(255, int(round(0.977 * vlc_lead_kmh + 0.752)))),
      "ACC_LeadSpeed":         vlc_lead_kmh,
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
