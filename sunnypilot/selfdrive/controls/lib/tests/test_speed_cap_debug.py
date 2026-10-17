from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_cap_debug import SPEED_CAP_KPH, apply_speed_cap


def test_cap_limits_cruise_above():
  assert apply_speed_cap((SPEED_CAP_KPH + 10) * CV.KPH_TO_MS) == SPEED_CAP_KPH * CV.KPH_TO_MS


def test_cap_never_raises_target():
  assert apply_speed_cap(50 * CV.KPH_TO_MS) == 50 * CV.KPH_TO_MS
  assert apply_speed_cap(0.0) == 0.0  # forceDecel / stop targets pass through
