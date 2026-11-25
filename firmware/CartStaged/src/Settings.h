#pragma once
#include <stdint.h>

// Set the stage in CartStaged.ino. Keep motor power disconnected through all
// six bench stages. Flash the matching protocol-3 dongle when updating protocol.
#ifndef CART_TEST_STAGE
#define CART_TEST_STAGE 1
#endif
#ifndef CART_EXPERIMENT_PROFILE
#define CART_EXPERIMENT_PROFILE 1 // 0: legacy regression semantics; 1: bounded bench/odometry
#endif

// 1 = radio / zero ACK, in loop (no PWM, sensors or control task)
// 2 = add 100 Hz control task (nonzero commands refused)
// 3 = add neutral ESC PWM on GPIO18 (nonzero commands refused)
// 4 = add right-wheel interrupt on GPIO34 (nonzero commands refused)
// 5 = add track interrupt on GPIO35 (nonzero commands refused)
// 6 = complete controller; nonzero commands can generate drive PWM
static_assert(CART_TEST_STAGE >= 1 && CART_TEST_STAGE <= 6, "Stage must be 1..6");

#ifndef CART_ENABLE_EXPERIMENT_LOG
#define CART_ENABLE_EXPERIMENT_LOG 1
#endif

namespace settings {
constexpr unsigned STAGE = CART_TEST_STAGE;
constexpr bool CONTROL_TASK = STAGE >= 2;
constexpr bool PWM = STAGE >= 3;
constexpr bool WHEEL_ISR = STAGE >= 4;
constexpr bool TRACK_ISR = STAGE >= 5;
constexpr bool DRIVE = STAGE == 6;
constexpr bool BENCH = CART_EXPERIMENT_PROFILE == 1;
constexpr bool EXPERIMENT_LOG = CART_ENABLE_EXPERIMENT_LOG && BENCH && CONTROL_TASK;
constexpr unsigned LOG_US = 20000; // 50 Hz logging; PI still 100 Hz
constexpr unsigned LOG_QUEUE_LENGTH = 64;
constexpr int ESC_PIN = 18;
constexpr int WHEEL_PIN = 34; // Physical right ISR input; retained for diagnostics.
constexpr int LEFT_WHEEL_PIN = 32;
constexpr bool LEFT_FEEDBACK = BENCH;
constexpr int FEEDBACK_WHEEL_PIN = LEFT_FEEDBACK ? LEFT_WHEEL_PIN : WHEEL_PIN;
constexpr int TRACK_PIN = 35; // Confirmed by user's independent working sketch.
constexpr unsigned CONTROL_US = 10000;
constexpr unsigned TELEMETRY_MS = 50;
constexpr unsigned QUEUE_LENGTH = 16;
constexpr unsigned PWM_HZ = 400; // Unpowered stage-6 timing experiment; ESC compatibility pending.
constexpr unsigned PWM_BITS = 16;
constexpr int NEUTRAL_US = 1500, MIN_US = 1000, MAX_US = 2000;
constexpr uint8_t DONGLE_MAC[6] = {0x88,0x56,0xA6,0x58,0x49,0x38};
constexpr uint8_t WIFI_CHANNEL = 1;
} // namespace settings
