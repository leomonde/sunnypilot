from math import cos, sin
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volvo.values import CANBUS, DBC

# Delphi ESR 2.5: 64 tracks at 0x500..0x53F, 20Hz, on CANBUS.body. Verified
# against rlog 0000003e seg 7 (Track 24 follows stock FSM1 ACC_Distance to the meter).
DELPHI_ESR_TRACK_ADDRS = list(range(0x500, 0x540))  # Target1..Target64
DELPHI_ESR_TRACK_NAMES = [f"Target{i}" for i in range(1, 65)]
TRACK_ADDR_NAMES = list(zip(DELPHI_ESR_TRACK_ADDRS, DELPHI_ESR_TRACK_NAMES, strict=True))
TRIGGER_MSG_ADDR = DELPHI_ESR_TRACK_ADDRS[-1]

INVALID_STATUSES = {0, 6}  # CAN_TX_TRACK_STATUS: 0=no target, 6=invalid coasted
MIN_VALID_CNT = 3  # consecutive valid frames before surfacing to planner


def _create_radar_can_parser(CP) -> CANParser:
  messages = [(name, 20) for name in DELPHI_ESR_TRACK_NAMES]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.body)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.updated_messages: set[int] = set()
    self.valid_cnt: dict[int, int] = {addr: 0 for addr in DELPHI_ESR_TRACK_ADDRS}
    self.track_id = 0
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    self.updated_messages.update(self.rcp.update(can_strings))

    # ESR sends 64 slots every 50ms — wait for last slot to publish a full sweep.
    if TRIGGER_MSG_ADDR not in self.updated_messages:
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for addr, name in TRACK_ADDR_NAMES:
      cpt = self.rcp.vl[name]
      rng = cpt["CAN_TX_TRACK_RANGE"]
      status = int(cpt["CAN_TX_TRACK_STATUS"])
      valid = status not in INVALID_STATUSES and rng > 0.5

      if valid:
        self.valid_cnt[addr] = min(self.valid_cnt[addr] + 1, MIN_VALID_CNT + 2)
      else:
        self.valid_cnt[addr] = max(self.valid_cnt[addr] - 1, 0)

      # Write track only when current sweep is valid (avoid leaking "no target" 0s
      # that saturate vRel to ±81.91 while counter still holds from prior frames).
      if valid and self.valid_cnt[addr] >= MIN_VALID_CNT:
        if addr not in self.pts:
          self.pts[addr] = structs.RadarData.RadarPoint()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1
        angle_rad = cpt["CAN_TX_TRACK_ANGLE"] * CV.DEG_TO_RAD
        # OP convention: dRel forward, yRel left+. ESR angle is right+, so flip sin().
        self.pts[addr].dRel = rng * cos(angle_rad)
        self.pts[addr].yRel = -rng * sin(angle_rad)
        self.pts[addr].vRel = cpt["CAN_TX_TRACK_RANGE_RATE"]
        self.pts[addr].aRel = cpt["CAN_TX_TRACK_RANGE_ACCEL"]
        self.pts[addr].yvRel = float("nan")
        self.pts[addr].measured = True
      elif self.valid_cnt[addr] < MIN_VALID_CNT and addr in self.pts:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
