// =====================================================================
//  R1 – ONE ESP32 for everything
//  Merges:  r1-controller (Bluepad32 gamepad, ESCs, upper, valves)
//         + r1-servo      (steering motors, encoders, PID, I2C slave)
//  The I2C link is gone; steering targets are passed in memory to a
//  100 Hz control task (control.h).
//
//  Board package : esp32_bluepad32 (Tools > Board > ESP32 Dev Module)
//  No extra libraries needed (ESC pulses are generated with LEDC directly,
//  so the Servo library is no longer used).
//
//  Pins / tuning : config.h      Wiring : WIRING.md
// =====================================================================
#include <Bluepad32.h>
#include <math.h>

// Wrong board package check. Arduino IDE shows TWO "ESP32 Dev Module" entries:
// you need the one under "esp32_bluepad32", not the one under "esp32" (Espressif).
// Also uninstall the "Bluepad32" library from Library Manager: that one is for
// NINA boards (Nano RP2040 Connect, MKR WiFi 1010...), not for ESP32.
#ifndef CONFIG_BLUEPAD32_MAX_DEVICES
#error "Wrong board: Tools > Board > esp32_bluepad32 > ESP32 Dev Module (and remove the 'Bluepad32' library from Library Manager)"
#endif

#include "config.h"
#include "controller.h"
#include "control.h"

// ---- Debug / bring-up modes (enable at most one TEST_*) --------------
// #define TEST_ENCODER       // type "motor count" pairs in Serial Monitor to set steering targets
// #define TEST_STEER_STEP    // all steering targets jump 0 <-> 360 counts every 2 s (was TEST_MOTOR_ORDER)
// #define TEST_STEER_MOTORS  // open-loop sweep of each steering motor (was motor_test in r1-servo)
// #define PRINT_ENCODERS     // print encoder counts every 500 ms

Shared::Command cmd;                       // owned by loop(), published every cycle
float servoRadians[Cfg::WHEEL_COUNT] = {}; // last commanded steering angle per module

// ---------------------------------------------------------------------
// angle (rad) -> nearest encoder target from the previous target (shortest turn)
int nearestTarget(int prevTarget, float radians, int cpr) {
  int targetCount = (int)roundf(radians / (2.0f * PI) * cpr);
  int diff = targetCount - prevTarget;
  diff = ((diff % cpr) + cpr) % cpr;
  if (diff > cpr / 2) diff -= cpr;
  return prevTarget + diff;
}

void syncSteering() {
  for (int i = 0; i < Cfg::WHEEL_COUNT; i++)
    cmd.steerTarget[i] = nearestTarget(cmd.steerTarget[i], servoRadians[i], Cfg::COUNTS_PER_REV[i]);
}

// Controller lost: stop everything that moves, re-centre steering, keep valves as they are.
void failsafe() {
  for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
    cmd.escUs[i] = Cfg::ESC_MIN_US;
    cmd.steerTarget[i] = 0;
  }
  cmd.conveyor = 0;
  cmd.arm = 0;
}

// ---------------------------------------------------------------------
int prevDpad, prevX, prevY, prevA;
bool raise_, extend_, grab_, wlift_, wtilt_, wgrab1_, wgrab2_;

void gamepadControl() {
  ControllerPtr c = Ctl::controller;

  // ---- swerve drive: right stick = translate, left stick X = rotate
  int cx = c->axisRX();
  int cy = -c->axisRY();
  int crot = c->axisX();

  if (c->b()) {                          // B = stop + reset steering to 0
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      cmd.escUs[i] = Cfg::ESC_MIN_US;
      servoRadians[i] = 0.0f;
      cmd.steerTarget[i] = 0;
    }
  } else {
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      int motor = Cfg::MOTOR_ORDERS[i];
      float x = cx * Cfg::SPEED_MULTIPLIER + Cfg::ROT_DIRECTIONS[i][0] * Cfg::ROT_MULTIPLIER * crot;
      float y = cy * Cfg::SPEED_MULTIPLIER + Cfg::ROT_DIRECTIONS[i][1] * Cfg::ROT_MULTIPLIER * crot;

      float targetRadian = atan2f(-x, y);
      float magnitude = sqrtf(x * x + y * y);
      if (magnitude > Cfg::MAX_SPEED) magnitude = Cfg::MAX_SPEED;

      cmd.escUs[motor] = (int)roundf(Cfg::ESC_MIN_US + magnitude);
      if (magnitude >= Cfg::MIN_ACCEPTABLE_SPEED) servoRadians[motor] = targetRadian;
    }
    syncSteering();
  }

  // ---- upper: L1/R1 = conveyor, R2/L2 = arm
  int brake = c->brake();
  int throttle = c->throttle();
  bool r1 = c->r1() != 0;
  bool l1 = c->l1() != 0;
  cmd.conveyor = (l1 - r1) * Cfg::MAX_CONVEYOR_DUTY;
  cmd.arm = (throttle - brake) * Cfg::MAX_ARM_DUTY / 1020;

  // ---- pneumatics (toggle on press)
  int dpad = c->dpad();
  int btnX = c->x(), btnY = c->y(), btnA = c->a();

  if (btnA && !prevA) grab_ = !grab_;
  if (btnX && !prevX) extend_ = !extend_;
  if (btnY && !prevY) raise_ = !raise_;
  // Bluepad32 d-pad bits: up=1, down=2, right=4, left=8 (same values as r1-controller)
  constexpr int PAD_UP = 1, PAD_DOWN = 2, PAD_RIGHT = 4, PAD_LEFT = 8;
  if ((dpad & PAD_UP)    && !(prevDpad & PAD_UP))    wgrab2_ = !wgrab2_;
  if ((dpad & PAD_DOWN)  && !(prevDpad & PAD_DOWN))  wgrab1_ = !wgrab1_;
  if ((dpad & PAD_RIGHT) && !(prevDpad & PAD_RIGHT)) wtilt_ = !wtilt_;
  if ((dpad & PAD_LEFT)  && !(prevDpad & PAD_LEFT))  wlift_ = !wlift_;

  prevDpad = dpad; prevA = btnA; prevX = btnX; prevY = btnY;

  uint8_t v = 0;
  v |= raise_  << Cfg::BIT_RAISE;
  v |= extend_ << Cfg::BIT_EXTEND;
  v |= grab_   << Cfg::BIT_GRAB;
  v |= wlift_  << Cfg::BIT_WLIFT;
  v |= wtilt_  << Cfg::BIT_WTILT;
  v |= wgrab1_ << Cfg::BIT_WGRAB1;
  v |= wgrab2_ << Cfg::BIT_WGRAB2;
  if (v != cmd.valves) Serial.printf("Valves: %02x\n", v);
  cmd.valves = v;
}

// ---------------------------------------------------------------------
#ifdef TEST_STEER_STEP
void testSteerStep(float dt) {
  static float timer = 0;
  for (int i = 0; i < Cfg::WHEEL_COUNT; i++) cmd.steerTarget[i] = timer < 2.0f ? 0 : 360;
  timer += dt;
  if (timer >= 4.0f) timer -= 4.0f;
}
#endif

#ifdef TEST_ENCODER
// Line format: "<motor> <count> [<motor> <count> ...]"
void testEncoder() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  int vals[16], n = 0;
  for (unsigned idx = 0; idx < line.length() && n < 16;) {
    while (idx < line.length() && !(isdigit(line[idx]) || line[idx] == '-')) idx++;
    if (idx >= line.length()) break;
    vals[n++] = line.substring(idx).toInt();
    idx++;
    while (idx < line.length() && isdigit(line[idx])) idx++;
  }
  for (int k = 0; k + 1 < n; k += 2) {
    if (vals[k] < 0 || vals[k] >= Cfg::WHEEL_COUNT) { Serial.println("Motor index must be 0..3."); continue; }
    cmd.steerTarget[vals[k]] = vals[k + 1];
  }
  Serial.println("Synced.");
}
#endif

#ifdef TEST_STEER_MOTORS
void testSteerMotors() {
  cmd.steerOpenLoop = true;
  for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
    Serial.printf("steer motor: %d\n", i);
    for (int s = 0; s < 256; s++)     { cmd.steerDuty[i] = s; Shared::publish(cmd); delay(4); }
    for (int s = 255; s > -256; s--)  { cmd.steerDuty[i] = s; Shared::publish(cmd); delay(4); }
    for (int s = -255; s <= 0; s++)   { cmd.steerDuty[i] = s; Shared::publish(cmd); delay(4); }
    delay(1000);
  }
}
#endif

// ---------------------------------------------------------------------
uint32_t elapsed;

void setup() {
  Serial.begin(115200);
  cmd = Shared::safeDefault();
  Control::setup();     // shift register, PWM, encoders, 100 Hz control task
  Ctl::setup();         // Bluetooth gamepad
  elapsed = millis();
  Serial.println("R1 single-controller ready.");
}

void loop() {
  delay(10);
  uint32_t now = millis();
  float dt = (now - elapsed) * 0.001f;
  elapsed = now;

#if defined(TEST_ENCODER)
  testEncoder();
#elif defined(TEST_STEER_STEP)
  testSteerStep(dt);
#elif defined(TEST_STEER_MOTORS)
  testSteerMotors();
#else
  (void)dt;
  if (Ctl::update()) gamepadControl();
  else failsafe();
#endif

  Shared::publish(cmd);

#ifdef PRINT_ENCODERS
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("Encoders: %d, %d, %d, %d | targets: %d, %d, %d, %d\n",
                  Control::measured[0], Control::measured[1], Control::measured[2], Control::measured[3],
                  cmd.steerTarget[0], cmd.steerTarget[1], cmd.steerTarget[2], cmd.steerTarget[3]);
  }
#endif
}
