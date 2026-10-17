"""
Test-only cap on the openpilot cruise speed, used to check that the car follows openpilot's
longitudinal commands and not the stock ACC logic (e.g. ACC set to 80 km/h, car must hold 70).

Enabled by writing the speed in km/h to CAP_PATH (e.g. `echo 70 > /data/oplong_cap_kph`) and
disabled by deleting the file, so it can be toggled on the device without a rebuild.
"""
import os

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

CAP_PATH = os.environ.get("OPLONG_CAP_PATH", "/data/oplong_cap_kph")
READ_INTERVAL = 20  # planner frames (DT_MDL 0.05s -> 1s)


def read_cap_kph(path: str = CAP_PATH) -> float | None:
  try:
    with open(path) as f:
      kph = float(f.read().strip())
  except (OSError, ValueError):
    return None
  return kph if 0. < kph < 255. else None


class SpeedCapDebug:
  def __init__(self, path: str = CAP_PATH):
    self.path = path
    self.cap_kph: float | None = None
    self.frame = 0

  def update(self) -> None:
    if self.frame % READ_INTERVAL == 0:
      cap_kph = read_cap_kph(self.path)
      if cap_kph != self.cap_kph:
        cloudlog.warning(f"speed_cap_debug: cap changed from {self.cap_kph} to {cap_kph} km/h")
      self.cap_kph = cap_kph
    self.frame += 1

  def apply(self, v_cruise: float) -> float:
    if self.cap_kph is None:
      return v_cruise
    return min(v_cruise, self.cap_kph * CV.KPH_TO_MS)
