"""
Development-only cap on the openpilot cruise speed (fork branch volvo_oplong).

Holds openpilot below the stock ACC set speed (e.g. ACC set to 80 km/h, car must hold 70) to check
that the car follows openpilot's longitudinal commands and not the stock camera logic.
To drive without the cap, use the volvo branch. Remove this module and its call in
selfdrive/controls/lib/longitudinal_planner.py once openpilot longitudinal is validated.
"""
from openpilot.common.constants import CV

SPEED_CAP_KPH = 70.


def apply_speed_cap(v_cruise: float) -> float:
  return min(v_cruise, SPEED_CAP_KPH * CV.KPH_TO_MS)
