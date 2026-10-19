from cereal import car
from openpilot.sunnypilot.selfdrive.controls.lib.follow_distance import VOLVO_T_FOLLOW, get_t_follow


def make_cp(brand: str, op_long: bool):
  return car.CarParams.new_message(brand=brand, openpilotLongitudinalControl=op_long)


def test_volvo_op_long_uses_fixed_gap():
  assert get_t_follow(make_cp("volvo", True)) == VOLVO_T_FOLLOW


def test_others_keep_personality():
  assert get_t_follow(make_cp("volvo", False)) is None
  assert get_t_follow(make_cp("toyota", True)) is None
