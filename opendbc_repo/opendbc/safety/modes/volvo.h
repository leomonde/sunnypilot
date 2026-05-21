#pragma once

#include "opendbc/safety/safety_declarations.h"

// Safety-relevant CAN messages for EUCD platform.
#define VOLVO_EUCD_AccPedal      0x020  // RX, gas pedal
#define VOLVO_EUCD_FSM0          0x051  // RX from FSM, cruise state
#define VOLVO_EUCD_VehicleSpeed1 0x148  // RX, vehicle speed
#define VOLVO_EUCD_Brake_Info    0x20a  // RX, driver brake pressed
#define VOLVO_EUCD_CCButtons     0x127  // TX by OP, CC buttons
#define VOLVO_EUCD_PSCM1         0x246  // TX by OP to camera, PSCM state
#define VOLVO_EUCD_FSM1          0x260  // TX by OP, ACC radar/distance message (oplong)
#define VOLVO_EUCD_FSM2          0x262  // TX by OP, LKA command
#define VOLVO_EUCD_FSM3          0x270  // TX by OP, ACC accel request + status
#define VOLVO_EUCD_FSM4          0x31A  // TX by OP, virtual lead car speed (oplong)

// CAN bus numbers.
#define VOLVO_MAIN_BUS 0U
#define VOLVO_AUX_BUS  1U
#define VOLVO_CAM_BUS  2U

static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
    {VOLVO_EUCD_CCButtons, VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,     VOLVO_CAM_BUS,  8, .check_relay = true},   // OP replaces stock steering servo state
    {VOLVO_EUCD_FSM2,      VOLVO_MAIN_BUS, 8, .check_relay = true},   // OP replaces stock LKA command
    // FSM1 / FSM3: DO NOT block forwarding. Stock cam FSM1/FSM3 carry a
    // 5-frame rolling counter pattern the car's ECM validates; intercepting
    // and replaying with passthrough delay causes the ECM to fault out after
    // ~30s (observed in drive 27 seg 0). Instead we allow stock to flow
    // cam->main untouched, and OP overlays its own FSM3 only when long-active.
    // Car's ECM gets both on main bus interleaved; OP's later arrival
    // dominates via last-message-wins.
    {VOLVO_EUCD_FSM1,      VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_FSM3,      VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_FSM4,      VOLVO_MAIN_BUS, 8, .check_relay = false},
  };

  // TODO: add counters
  static RxCheck volvo_eucd_rx_checks[] = {
    {.msg = {{VOLVO_EUCD_AccPedal,      VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_FSM0,          VOLVO_CAM_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_VehicleSpeed1, VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_Brake_Info,    VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

static void volvo_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == VOLVO_MAIN_BUS) {
    if (msg->addr == VOLVO_EUCD_VehicleSpeed1) {
      // Signal: VehicleSpeed
      unsigned int speed_raw = (GET_BYTES(msg, 6, 1) << 8) | GET_BYTES(msg, 7, 1);
      vehicle_moving = speed_raw >= 36U;
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01 / 3.6);
    }

    if (msg->addr == VOLVO_EUCD_AccPedal) {
      // Signal: AccPedal
      unsigned int gas_raw = ((GET_BYTES(msg, 2, 1) & 0x03U) << 8) | GET_BYTES(msg, 3, 1);
      gas_pressed = gas_raw >= 100U;
    }

    if (msg->addr == VOLVO_EUCD_Brake_Info) {
      // Signal: BrakePedal
      brake_pressed = ((GET_BYTES(msg, 2, 1) & 0x0CU) >> 2U) == 2U;
    }
  } else if (msg->bus == VOLVO_CAM_BUS) {
    if (msg->addr == VOLVO_EUCD_FSM0) {
      // Signal: ACC_Enabled (bit 2 of byte 2, from ACCStatus == 6 || 7)
      bool cruise_engaged = (GET_BYTES(msg, 2, 1) & 0x04U) != 0U;
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  // Longitudinal safety limits — raw byte 1 of FSM3 is ACC_AccelerationRequest
  // encoded as (0.04, -5.04). Safety-side we check raw_accel = byte1 - 126.
  // +50 raw =>  +2.0 m/s^2 max accel
  // -100 raw => -4.0 m/s^2 max decel
  const LongitudinalLimits VOLVO_LONG_LIMITS = {
    .max_accel = 50,
    .min_accel = -100,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool violation = false;

  // Safety check for CC button signals.
  if (msg->addr == VOLVO_EUCD_CCButtons) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    violation |= !cruise_engaged_prev && (GET_BIT(msg, 59U) || !(GET_BIT(msg, 43U)));  // Signals: ACCOnOffBtn, ACCOnOffBtnInv (cancel)
    violation |= !controls_allowed && (GET_BIT(msg, 61U) || !(GET_BIT(msg, 45U)));  // Signals: ACCResumeBtn, ACCResumeBtnInv (resume)
  }

  // Safety check for Lane Keep Assist action.
  if (msg->addr == VOLVO_EUCD_FSM2) {
    // Signal: LKASteerDirection
    unsigned int mode = GET_BYTES(msg, 5, 1) & 0x03U;
    bool lka_active = mode != 0U;

    if (lka_active && !controls_allowed) {
      violation = true;
    }
  }

  // Longitudinal control: gate on controls_allowed + range check.
  // With FSM3 check_relay=false, stock flows cam->main uninterrupted.
  // OP only TXs FSM3 when actively controlling long (CC.longActive), so
  // controls_allowed will be true whenever OP's FSM3 reaches this hook.
  if (msg->addr == VOLVO_EUCD_FSM3) {
    int raw_accel = (int)GET_BYTES(msg, 1, 1) - 126;
    if (!controls_allowed || longitudinal_accel_checks(raw_accel, VOLVO_LONG_LIMITS)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool volvo_fwd_hook(int bus_num, int addr) {
  // Block stock FSM1/FSM3 from cam->main when OP is in control, so OP can
  // relay them at 50Hz (passthrough) and override ACC_Check=1 during SNG
  // without stock's ACC_Check=0 overwriting OP's value on the bus.
  if (bus_num == VOLVO_CAM_BUS && controls_allowed && !gas_pressed) {
    if (addr == VOLVO_EUCD_FSM1 || addr == VOLVO_EUCD_FSM3 || addr == VOLVO_EUCD_FSM4) {
      return true;  // block forwarding; OP relays (with virtual lead when longActive)
    }
  }
  return false;
}

static safety_config volvo_init(uint16_t param) {
  (void)param;

  return BUILD_SAFETY_CFG(volvo_eucd_rx_checks, VOLVO_EUCD_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  .fwd = volvo_fwd_hook,
};
