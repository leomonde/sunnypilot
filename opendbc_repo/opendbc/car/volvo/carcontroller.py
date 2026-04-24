import numpy as np
from opendbc.can import CANPacker
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CANBUS, CarControllerParams, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE

    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    # Frame when the most recent resume-blast cycle started. Used to pass
    # through stock's FSM3 accel during the take-off window so the ECM's
    # actual response matches stock ACC's internal expectation — OP's
    # planner under-commands and stock cancels (drives 3e seg 7, 3f seg 11).
    self.takeoff_start_frame = -1_000_000

    # Next wall-clock nanosecond at which FSM3 / FSM1 TX is allowed.
    # controlsd scheduling jitter means self.frame % 2 == 0 gives us 10–30ms
    # intervals instead of a clean 20ms (drive 41 seg 4: OP TX stdev 2.6ms
    # vs stock 0.5ms, with 9.8ms bursts and 30ms gaps). The ECM validates
    # cadence of stock ACC messages — our bursty pattern looks like a fault
    # and likely contributes to the self-cancels. Gate TX on wall-clock.
    self.next_long_tx_nanos = 0
    # Period between FSM3 TXs (20ms = 50Hz, matching stock).
    self.LONG_TX_PERIOD_NANOS = 20_000_000

    # ACC_Check is the resume-button acknowledgement bit in FSM3. Per
    # leomonde, it must be forced to 1 specifically during the resume
    # blast (not copied from stock's FSM3, which sits at 0 almost always).
    # Count of remaining FSM3 TXs that should carry ACC_Check=1. Set to 25
    # (~0.5s of 50Hz TX) when SNG fires a resume blast; decrements to 0.
    self.sng_ack_frames = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    accel = 0.0  # always defined so SNG block never NameErrors

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # Cancel ACC if engaged when OP is not, but only above minimum steering speed.
    # TODO: is this check needed? it might trying to fix broken standstill behavior
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # run at 50hz
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        #apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CarControllerParams)
        apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        # Update prev with desired if just enabled.
        if not self.latActive_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
            self.steer_blocked = False
        else:
          if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
            self.steer_blocked = True
            self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
            self.steer_dir_bf_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          # Set old request when inside deadzone
          apply_steer_dir = self.apply_steer_dir_prev

      else:
        apply_steer = 0
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.latActive_prev = CC.latActive

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)

    # SNG — evaluated BEFORE the long-control TX block so the take-off window
    # flag set here is visible when we pick OP-vs-stock accel below.
    # wait 100 cycles since last resume sent
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      # Trigger resume on lead moving OR on planner clearing shouldStop (green
      # light / stop sign cleared in experimental mode). CC.cruiseControl.resume
      # is True when OP is engaged, car is at standstill, and shouldStop → False.
      lead_moved = CS.acc_distance > self.distance
      e2e_resume = CC.cruiseControl.resume

      if at_standstill and self.waiting and (lead_moved or e2e_resume):
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        if self.CP.openpilotLongitudinalControl:
          # Already sending FSM3 every frame above; SNG just needs the resume button blast.
          pass
        else:
          can_sends.extend([volvocan.create_acc_state_msg(self.packer_pt)] * 25)
        # Mark the start of the take-off window on the first blast of this
        # resume cycle so the long block below can defer to stock's accel.
        # Also arm the ACC_Check=1 acknowledgement window for the next
        # ~0.5s of FSM3 TXs — the car needs that ack to actually honor the
        # resume button (per leomonde: "force 1, not copy from FSM").
        if self.sng_count == 0:
          self.takeoff_start_frame = self.frame
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # Longitudinal: only TX when OP is actively controlling (longActive).
    # Panda's fwd hook now unblocks stock FSM1/FSM3 when !gas_pressed, so
    # during gas overrides stock flows through naturally and there's no
    # starvation even though OP isn't TXing. When longActive flickers False
    # due to gas press, OP withdraws from the bus entirely and the car
    # sees stock's real-time FSM3 — matching pre-OP-long behavior.
    #
    # Rate gating: wall-clock 20ms minimum between TXs, so controlsd jitter
    # can't produce the 10ms bursts leomonde spotted. If we're late we still
    # TX once and slide the next-allowed forward by one period (no catch-up
    # burst).
    long_tx_due = self.CP.openpilotLongitudinalControl and CC.longActive and now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      # Advance target by exactly one period. If we'd already be past the
      # advanced time (first TX after longActive gap, or controlsd stalled
      # for >1 period), resync to avoid a burst of catch-up TXs.
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      op_accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Take-off passthrough: after a SNG resume blast, while still rolling
      # below 5 m/s, defer to stock's ACC_AccelerationRequest when stock
      # wants MORE positive accel than OP. Stock's take-off ramp was
      # commanding +0.88 m/s² in drive 0000042 seg 3 while OP only wanted
      # +0.62; the car lagged stock's expected response profile and stock
      # cancelled 0.3s into the takeoff. Mirroring stock's value (when
      # higher) keeps stock's state machine satisfied. OP takes back over
      # once cruising above 5 m/s.
      #
      # Previously we gated on a 3s time window starting from the resume
      # blast, but that window expired mid-takeoff when the resume hit near
      # a segment boundary (drive 0000042 seg 2→3). vEgo threshold is a
      # more reliable trigger.
      #
      # Direction guard: only passes stock's POSITIVE accel. If OP wants to
      # brake during the takeoff (e.g. lead suddenly stopped), OP's value
      # drives — OP sees the lead via radar, stock's brake authority via
      # FSM3 is weak on this car anyway.
      takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
      stock_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      # 15s ceiling is a safety belt in case vEgo never crosses 5 (crawl
      # traffic) — eventually snap back to OP so runaway stock commands
      # can't persist indefinitely.
      in_takeoff_window = takeoff_elapsed < 15.0 and CS.out.vEgo < 5.0
      if in_takeoff_window and stock_accel > op_accel and stock_accel > 0:
        accel = stock_accel
      else:
        accel = op_accel

      # ACC_Check: 1 only during the post-resume acknowledgement window
      # (set by the SNG block above), else 0. Copying stock's ACC_Check —
      # as we were doing — left it at 0 almost always, so SNG resumes
      # worked only when the stock cam-bus value happened to flip in time.
      acc_check = 1 if self.sng_ack_frames > 0 else 0
      if self.sng_ack_frames > 0:
        self.sng_ack_frames -= 1

      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check))
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, True))

    # FSM3/FSM1 are TX'd together inside long_tx_due so they stay in the
    # same bus frame and share the 50Hz cadence.


    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, self.packer_pt, self.frame, self.last_button_frame))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
