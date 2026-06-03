"""
Volvo SLA (Speed Limit Assist) — in-opendbc ICBM-style implementation.

Pattern: "Drive-at-limit" armed by the engagement button, with single-press disarm.
  When ACC is engaged VIA set+/set- (the SET buttons), SLA arms and drives the setpoint
  to the limit — UP or DOWN. When ACC is engaged VIA resume, the setpoint is restored but
  SLA stays OFF (plain manual cruise). When the limit clears, SLA stops acting (keeps
  whatever setpoint is current). If the driver presses set+/set- manually, SLA disarms
  permanently until the next ACC engagement.

Speed limit source:
  Carcontroller passes mainline's POLICY-RESOLVED speed limit (TSR+map combined per
  the user's SpeedLimitControlPolicy UI choice: TSR-first / map-first / TSR-only /
  map-only). Falls back to raw TSR if no resolved value is available. The argument
  is named `tsr_kph` for backward compat but represents the resolved value.

Disarm:
  - Any physical set+/set- press by the driver → disarm for the rest of this session.
    Detected from the real CEM CCButtons (carstate), so it works even while SLA is
    actively pressing — unlike inferring it from setpoint changes.
  - Re-arm on next ACC engagement only if engaged via set+/set-; engaging via resume
    keeps SLA disarmed (manual cruise at the restored setpoint).

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
  # How far back (frames) to look for the driver's engagement button when ACC turns on.
  # The set/resume press lands ~90ms before ACC_Enabled flips; this window attributes it.
  ENGAGE_ATTRIBUTION_FRAMES = 150  # 1.5s at 100Hz

  def __init__(self):
    self.acc_was_on = False
    self.session_disabled = False
    self.last_press_frame = -1000
    # Frame of the last PHYSICAL driver press (from CEM CCButtons, not our injected presses)
    self.last_driver_adjust_frame = -100000   # set+ (ACCSetBtn) or set- (ACCMinusBtn)
    self.last_driver_resume_frame = -100000   # resume (ACCResumeBtn)

  def update(self, tsr_kph: float, setpoint_kph: float, vEgo_kph: float,
             acc_on: bool, frame: int, sla_mode: int,
             driver_adjust_press: bool = False, driver_resume_press: bool = False) -> str | None:
    """Returns 'set-' | 'set+' | None.

    driver_adjust_press: rising edge of a physical set+/set- press by the driver.
    driver_resume_press:  rising edge of a physical resume press by the driver.
    sla_mode: SpeedLimitMode param value (0=off, 1=info, 2=warning, 3=assist).
              We only emit presses when assist; other modes track state silently.
    """
    # Track physical driver presses (independent of our injected presses, which never
    # appear on the RX bus the carstate parser reads).
    if driver_adjust_press:
      self.last_driver_adjust_frame = frame
    if driver_resume_press:
      self.last_driver_resume_frame = frame

    # On new ACC engagement, choose arm behavior by HOW it was engaged:
    #   engaged via set+/set- → arm SLA (auto-follow the limit)
    #   engaged via resume    → restore the setpoint but keep SLA off (manual cruise)
    just_engaged = acc_on and not self.acc_was_on
    if just_engaged:
      engaged_via_resume = (self.last_driver_resume_frame > self.last_driver_adjust_frame and
                            frame - self.last_driver_resume_frame <= self.ENGAGE_ATTRIBUTION_FRAMES)
      self.session_disabled = engaged_via_resume
    self.acc_was_on = acc_on

    if not acc_on or self.session_disabled or sla_mode != SLA_MODE_ASSIST:
      return None

    # Any physical set+/set- press by the driver disarms for the rest of the session.
    # Read from the real CEM button, so it works even while we are actively pressing.
    # Exclude the very press that just engaged ACC (that one arms, it must not disarm).
    if driver_adjust_press and not just_engaged:
      self.session_disabled = True
      return None

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
