"""
[fork-only, test] Virtual red light: while carState.genericToggle is on (Volvo: high beam or wipers)
in experimental mode, a stopped obstacle is placed ahead so OP stops to 0 km/h. Toggle off = green.
"""
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL

ON_TIME = 0.5     # s the toggle must stay on (ignores a single wipe)
DECEL = 1.5       # m/s², sets the virtual light distance at the trigger
MIN_DIST = 20.    # m


class VirtualRedLight:
  def __init__(self):
    self.on_frames = 0
    self.dist = None  # m to the virtual stop line, None = off

  def update(self, sm, v_ego):
    radar = sm['radarState']
    toggle = sm['carState'].genericToggle and sm['selfdriveState'].experimentalMode and sm['selfdriveState'].enabled
    self.on_frames = self.on_frames + 1 if toggle else 0

    if self.on_frames * DT_MDL < ON_TIME:
      self.dist = None
      return radar

    if self.dist is None:
      self.dist = max(v_ego ** 2 / (2 * DECEL), MIN_DIST)
    else:
      self.dist = max(self.dist - v_ego * DT_MDL, 0.)

    # virtual light as leadOne, real lead kept as leadTwo
    light = SimpleNamespace(status=True, dRel=self.dist, vLead=0., aLeadK=0., aLeadTau=0., modelProb=0.)
    return SimpleNamespace(leadOne=light, leadTwo=radar.leadOne)
