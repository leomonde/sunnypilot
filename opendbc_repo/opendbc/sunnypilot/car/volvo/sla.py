"""
Volvo SLA (Speed Limit Assist) — in-opendbc ICBM-style implementation.

Pattern: auto-arm "Drive-at-limit" with single-press disarm.
  When ACC is engaged, SLA arms automatically. When TSR detects a sign, SLA presses
  set-/set+ to drive the setpoint to the limit — UP or DOWN. When the sign clears,
  SLA stops acting (keeps whatever setpoint is current). If the driver presses
  set+/set- manually, SLA disarms permanently until the next ACC engagement.

Disarm:
  - Any manual press by driver → disarm for the rest of this session
  - Reset on next ACC engagement (driver disengages + reengages ACC)

UI mode respect: only acts when SpeedLimitMode param == 3 (assist). For other
modes (off/information/warning), SLA tracks state but emits no presses.

Coexistence with mainline sunnypilot SLA:
  Mainline SLA runs the non-pcm_op_long state machine for Volvo (via the
  speed_limit_assist.py one-line patch). Its ICBM stays off because oplong is on,
  so mainline doesn't actually press anything — our SLA does. UI alerts from
  mainline still appear normally.
"""


# SpeedLimitMode values from sunnypilot/selfdrive/controls/lib/speed_limit/common.py
SLA_MODE_ASSIST = 3  # only mode where our SLA emits presses


class VolvoSlaController:
  # Press cadence (Volvo accepts ~600ms between presses comfortably)
  PRESS_INTERVAL_FRAMES = 30
  # Only act if setpoint vs target differs by more than this
  ENGAGE_THRESHOLD_KPH = 3
  # Min speed for SLA to act at all
  MIN_VEGO_KPH = 30
  # Window after our press where setpoint change is attributed to us, not driver.
  # Must cover Volvo's full processing latency for the button → ACC_Speed update
  # (observed up to ~600ms). Too short = SLA falsely disarms on its own presses.
  OUR_PRESS_SUPPRESS_FRAMES = 80   # 800ms at 100Hz

  def __init__(self):
    self.acc_was_on = False
    self.session_disabled = False
    self.last_press_frame = -1000
    self.last_setpoint_kph = 0

  def update(self, tsr_kph: float, setpoint_kph: float, vEgo_kph: float,
             acc_on: bool, frame: int, sla_mode: int) -> str | None:
    """Returns 'set-' | 'set+' | None.

    sla_mode: SpeedLimitMode param value (0=off, 1=info, 2=warning, 3=assist).
              We only emit presses when assist; other modes track state silently.
    """
    # Reset on new ACC engagement (rearm even if previously disarmed)
    if acc_on and not self.acc_was_on:
      self.session_disabled = False
      self.last_setpoint_kph = setpoint_kph
    self.acc_was_on = acc_on

    if not acc_on or self.session_disabled or sla_mode != SLA_MODE_ASSIST:
      self.last_setpoint_kph = setpoint_kph
      return None

    # Manual press detection: setpoint changed outside our press window → disarm
    if setpoint_kph != self.last_setpoint_kph:
      our_press_recent = (frame - self.last_press_frame) <= self.OUR_PRESS_SUPPRESS_FRAMES
      if not our_press_recent:
        self.session_disabled = True
        self.last_setpoint_kph = setpoint_kph
        return None
    self.last_setpoint_kph = setpoint_kph

    # No TSR sign → nothing to do (keep current setpoint, don't restore anything)
    if tsr_kph <= 0:
      return None

    if vEgo_kph < self.MIN_VEGO_KPH:
      return None

    if frame - self.last_press_frame < self.PRESS_INTERVAL_FRAMES:
      return None

    # Drive setpoint to TSR limit — UP or DOWN
    diff = tsr_kph - setpoint_kph
    if diff <= -self.ENGAGE_THRESHOLD_KPH:
      self.last_press_frame = frame
      return 'set-'
    if diff >= self.ENGAGE_THRESHOLD_KPH:
      self.last_press_frame = frame
      return 'set+'
    return None
