from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import AngleSteeringLimits, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

class SteerDirection(IntEnum):
  # LKASteerDirection: EUCD needs 8-frame wait on direction change; C1MCA can use BOTH.
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )

  STEER_TIMEOUT = 30 / DT_CTRL  # frames of 0-torque EPS before steer fault
  BLOCK_LEN = 8                  # EUCD: frames to block steering on direction change
  DEADZONE = 0.2                 # deg: hold previous direction inside deadzone
  ACCEL_MIN = -4.0               # m/s²
  ACCEL_MAX = 2.0                # m/s²
  # Phase 1b: suppress OP negative accel near setpoint when stock isn't actively braking.
  # If stock_accel < STOCK_BRAKE_DEMAND, lead/AEB is dominant → OP must NOT be suppressed.
  ACC_SPEED_COHERENCE_MARGIN = 1.4  # m/s (~5 km/h around setpoint)
  STOCK_BRAKE_DEMAND = -0.3         # m/s² — threshold for "stock actively braking"
  # Sign-coherence: defer to stock when it's in a clearly positive transient
  # (engagement, coming off gas) — avoids ECM rejection from cross-msg incoherence.
  STOCK_POSITIVE_TRANSIENT = 0.1    # m/s² — threshold for "stock clearly positive"
  # Phase 2: with real lead, clamp OP accel within ±this of stock's accel.
  # Lead data passes through (real), OP only adjusts braking magnitude.
  BRAKE_CLAMP_MARGIN = 0.7  # m/s²

  def __init__(self, CP):
    pass  # CP currently unused; kept for API compatibility


class CANBUS:
  pt = 0
  body = 1  # aux bus (Delphi ESR 2.5 radar at 0x500..0x53F + CEM dynamics)
  cam = 2


RADAR_ESR = "ESR"  # Delphi ESR 2.5 — 64 tracks @ 20Hz on aux bus

@dataclass
class VolvoEUCDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_v60_2015_pt',
    Bus.radar: RADAR_ESR,
  })


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass(frozen=True)
class VolvoCarSpecs(CarSpecs):
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.44
  minSteerSpeed: float = 1.0


class CAR(Platforms):
  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [VolvoCarDocs("Volvo V60")],
    VolvoCarSpecs(mass=1750, wheelbase=2.776),
  )


VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
