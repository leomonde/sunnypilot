from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_cap_debug import READ_INTERVAL, SpeedCapDebug, read_cap_kph


def test_no_file_no_cap(tmp_path):
  cap = SpeedCapDebug(str(tmp_path / "cap"))
  cap.update()
  assert cap.cap_kph is None
  assert cap.apply(30.0) == 30.0


def test_cap_limits_cruise(tmp_path):
  path = tmp_path / "cap"
  path.write_text("70\n")
  cap = SpeedCapDebug(str(path))
  cap.update()
  assert cap.apply(80 * CV.KPH_TO_MS) == 70 * CV.KPH_TO_MS
  assert cap.apply(50 * CV.KPH_TO_MS) == 50 * CV.KPH_TO_MS  # never raises the target


def test_cap_toggles_on_reread(tmp_path):
  path = tmp_path / "cap"
  cap = SpeedCapDebug(str(path))
  cap.update()
  path.write_text("70")
  for _ in range(READ_INTERVAL - 1):
    cap.update()
  assert cap.cap_kph is None  # not re-read yet
  cap.update()
  assert cap.cap_kph == 70.0
  path.unlink()
  for _ in range(READ_INTERVAL):
    cap.update()
  assert cap.cap_kph is None


def test_invalid_values_ignored(tmp_path):
  path = tmp_path / "cap"
  for content in ("", "abc", "0", "-10", "300"):
    path.write_text(content)
    assert read_cap_kph(str(path)) is None
