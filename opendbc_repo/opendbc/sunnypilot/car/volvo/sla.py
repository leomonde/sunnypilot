"""
Volvo SLA (Speed Limit Assist) — in-opendbc ICBM-based.

Pattern: auto-arm "Setpoint-as-max" with double-press disarm.
  When ACC is engaged, SLA arms automatically. Driver's setpoint at engagement
  is captured as driver_max. When TSR detects a sign, SLA presses set- until
  setpoint matches the limit. When the sign clears or a higher limit appears,
  SLA presses set+ to restore driver_max.

  Manual override:
    - Single press of set+/set- → update driver_max + pause SLA 10s
    - Two presses within 3s → disarm SLA for the rest of this session
    - Reset on next ACC engagement (driver disengages + reengages ACC)

Coexistence with mainline sunnypilot SLA:
  Both can be on. With this SLA dropping the setpoint, the mainline SLA's brake
  intentions stay aligned with stock (because stock also brakes when setpoint
  drops). Phase 2 clamp ±BRAKE_CLAMP_MARGIN keeps OP within stock's envelope.
"""


class VolvoSlaController:
  # Press cadence (Volvo accepts ~600ms between presses comfortably)
  PRESS_INTERVAL_FRAMES = 30
  # Only act if setpoint vs target differs by more than this
  ENGAGE_THRESHOLD_KPH = 3
  # Min speed for SLA to act at all
  MIN_VEGO_KPH = 30
  # Window after our press where setpoint change is attributed to us, not driver
  OUR_PRESS_SUPPRESS_FRAMES = 25
  # After a single manual press, pause SLA this long to let the driver settle
  PAUSE_AFTER_MANUAL_FRAMES = 500
  # Two manual presses inside this window disarm SLA for the session
  DOUBLE_PRESS_WINDOW_FRAMES = 150

  def __init__(self):
    self.driver_max_kph = 0
    self.acc_was_on = False
    self.session_disabled = False
    self.last_press_frame = -1000
    self.last_manual_press_frame = -1000
    self.last_setpoint_kph = 0

  def update(self, tsr_kph: float, setpoint_kph: float, vEgo_kph: float,
             acc_on: bool, frame: int) -> str | None:
    """Returns 'set-' | 'set+' | None."""
    # Reset on new ACC engagement
    if acc_on and not self.acc_was_on:
      self.driver_max_kph = setpoint_kph
      self.session_disabled = False
      self.last_setpoint_kph = setpoint_kph
      self.last_manual_press_frame = -1000
    self.acc_was_on = acc_on

    if not acc_on or self.session_disabled:
      self.last_setpoint_kph = setpoint_kph
      return None

    # Manual press detection: setpoint changed outside our press window
    if setpoint_kph != self.last_setpoint_kph:
      our_press_recent = (frame - self.last_press_frame) <= self.OUR_PRESS_SUPPRESS_FRAMES
      if not our_press_recent:
        if frame - self.last_manual_press_frame < self.DOUBLE_PRESS_WINDOW_FRAMES:
          # Second manual press within 3s → disarm for session
          self.session_disabled = True
          self.last_setpoint_kph = setpoint_kph
          return None
        # Single press → update driver_max, record press time
        self.driver_max_kph = setpoint_kph
        self.last_manual_press_frame = frame
    self.last_setpoint_kph = setpoint_kph

    # Pause after manual override
    if frame - self.last_manual_press_frame < self.PAUSE_AFTER_MANUAL_FRAMES:
      return None

    if vEgo_kph < self.MIN_VEGO_KPH:
      return None

    if frame - self.last_press_frame < self.PRESS_INTERVAL_FRAMES:
      return None

    # Target = limit (capped at driver_max) when TSR active, else restore driver_max
    if tsr_kph > 0:
      target_kph = min(tsr_kph, self.driver_max_kph)
    else:
      target_kph = self.driver_max_kph

    diff = target_kph - setpoint_kph
    if diff <= -self.ENGAGE_THRESHOLD_KPH:
      self.last_press_frame = frame
      return 'set-'
    if diff >= self.ENGAGE_THRESHOLD_KPH:
      self.last_press_frame = frame
      return 'set+'
    return None
