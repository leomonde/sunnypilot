"""
Following time gap for openpilot longitudinal on Volvo (fork branch volvo_oplong).

openpilot's personalities (1.25/1.45/1.75 s + 6 m) kept the car 5-15 m farther from the lead than the stock ACC
(routes 807/808 vs 824, camera radar distance at steady speed: stock about 1.0-1.3 s, OP standard 1.9 s). The stock
ACC sits close to t_follow 1.0 s in the same formula, so Volvo uses 1.0 s with every personality.
"""
from cereal import car

VOLVO_T_FOLLOW = 1.0  # s


def get_t_follow(CP: car.CarParams) -> float | None:
  """Time gap override for the long MPC, or None to keep the driving personality's."""
  if CP.brand == "volvo" and CP.openpilotLongitudinalControl:
    return VOLVO_T_FOLLOW
  return None
