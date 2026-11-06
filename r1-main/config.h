#pragma once
// =====================================================================
//  R1 – single-ESP32 controller : ALL pins and tuning in one place
//  Board: ESP32-WROOM-32 DevKit (NOT WROVER – GPIO16/17 are used)
//  Core : Bluepad32 board package (esp32_bluepad32, Arduino-ESP32 2.0.x)
// =====================================================================

namespace Cfg {
  constexpr int WHEEL_COUNT = 4;

  // ---------------- Drive wheels: 4x ESC (servo-style PWM) ------------
  // Physical module index 0..3 (same numbering as the old r1-servo board)
  constexpr int ESC_PINS[WHEEL_COUNT] = {4, 13, 19, 21};
  constexpr int ESC_MIN_US = 1000;      // stopped / armed
  constexpr int ESC_MAX_US = 2000;
  constexpr int ESC_FREQ   = 50;        // Hz
  constexpr int ESC_RES    = 16;        // bits

  // ---------------- Steering motors: 4x DC + H-bridge -----------------
  // PWM pins (direction INA/INB comes from the shift register, see below)
  constexpr int STEER_PWM_PINS[WHEEL_COUNT] = {17, 16, 27, 14};
  constexpr int MOTOR_PWM_FREQ = 20000; // Hz (steering + upper motors)
  constexpr int MOTOR_PWM_RES  = 8;     // bits -> duty 0..255
  constexpr int STEER_MAX_DUTY = 100;   // clamp (was MAX_SPEED in r1-servo)
  // +1 / -1 : flips the motor direction of a module
  constexpr int STEER_MOTOR_DIR[WHEEL_COUNT] = {1, 1, -1, 1};

  // ---------------- Steering encoders (quadrature, PCNT) --------------
  constexpr int ENC_A_PINS[WHEEL_COUNT] = {39, 35, 33, 26};
  constexpr int ENC_B_PINS[WHEEL_COUNT] = {36, 34, 32, 25};
  constexpr int ENC_DIR[WHEEL_COUNT]    = {-1, -1, -1, -1};
  constexpr int COUNTS_PER_REV[WHEEL_COUNT] = {4000, 4000, 4000, 4000}; // per steering rev

  // ---------------- Steering position PID ------------------------------
  constexpr float STEER_P = 0.2f;
  constexpr float STEER_I = 0.0f;
  constexpr float STEER_D = 0.0f;
  constexpr int   CONTROL_PERIOD_MS = 10;   // 100 Hz control loop

  // ---------------- Upper mechanism DC motors --------------------------
  constexpr int CONVEYOR_PWM_PIN = 22;
  constexpr int ARM_PWM_PIN      = 2;   // GPIO2 = on-board LED on most DevKits (harmless)
  constexpr int MAX_CONVEYOR_DUTY = 100;
  constexpr int MAX_ARM_DUTY      = 50;

  // ---------------- 74HC595 chain (3 chips = 24 outputs) ---------------
  constexpr int SR_SCLK  = 18;   // SHCP (pin 11) of all 595s
  constexpr int SR_MOSI  = 23;   // DS   (pin 14) of U1
  constexpr int SR_LATCH = 5;    // STCP (pin 12) of all 595s
  constexpr int SR_OE    = 0;    // /OE  (pin 13) of all 595s, 10k pull-up to 3V3. -1 = not wired
  constexpr int SR_CLOCK = 10000000;

  // Bit map of the 24-bit chain
  //   U1 (bits 0..7)   : pneumatic valves
  //   U2 (bits 8..15)  : upper motor H-bridge direction
  //   U3 (bits 16..23) : steering H-bridge direction
  constexpr int BIT_RAISE = 0, BIT_EXTEND = 1, BIT_GRAB = 2, BIT_WLIFT = 3,
                BIT_WTILT = 4, BIT_WGRAB1 = 5, BIT_WGRAB2 = 6;          // bit 7 spare
  constexpr int BIT_CONVEYOR_A = 8, BIT_CONVEYOR_B = 9,
                BIT_ARM_A = 10, BIT_ARM_B = 11;                          // 12..15 spare
  constexpr int STEER_INA_BITS[WHEEL_COUNT] = {16, 18, 22, 20};
  constexpr int STEER_INB_BITS[WHEEL_COUNT] = {17, 19, 23, 21};

  // ---------------- LEDC channels (pairs share a timer!) ---------------
  //  ch0-3  : ESCs        (50 Hz / 16 bit)  timers HS0, HS1
  //  ch4-5  : conveyor/arm(20 kHz / 8 bit)  timer  HS2
  //  ch6-9  : steering    (20 kHz / 8 bit)  timers HS3, LS0
  constexpr int ESC_CH[WHEEL_COUNT]   = {0, 1, 2, 3};
  constexpr int CONVEYOR_CH = 4, ARM_CH = 5;
  constexpr int STEER_CH[WHEEL_COUNT] = {6, 7, 8, 9};

  // ---------------- Swerve kinematics (from r1-controller) -------------
  // logical wheel i -> physical module MOTOR_ORDERS[i]
  constexpr int   MOTOR_ORDERS[WHEEL_COUNT]     = {2, 3, 1, 0};
  constexpr int   ROT_DIRECTIONS[WHEEL_COUNT][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
  constexpr float SPEED_MULTIPLIER = 0.5f;
  constexpr float ROT_MULTIPLIER   = 0.25f;
  constexpr float MAX_SPEED        = 300.0f;  // added to ESC_MIN_US
  constexpr float MIN_ACCEPTABLE_SPEED = 50.0f; // below this the wheel keeps its angle

  // ---------------- Gamepad failsafe -----------------------------------
  constexpr int CONTROLLER_TIMEOUT_MS = 1000;
}
