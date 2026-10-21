#pragma once
// Hardware control task (replaces the whole r1-servo board + the I2C link).
//
//   loop()  (gamepad, kinematics) --Shared::publish()-->  controlTask (100 Hz)
//                                                          |- read 4 encoders
//                                                          |- steering PID -> PWM + dir bits
//                                                          |- ESC pulses
//                                                          |- conveyor / arm PWM + dir bits
//                                                          |- valves
//                                                          '- one 24-bit shift-register write
//
// controlTask is the ONLY code that touches outputs, so there are no races
// on the shift register or LEDC.
#include <Arduino.h>
#include "config.h"
#include "pwm.h"
#include "shifter.h"
#include "encoder.h"

namespace Shared {
  struct Command {
    int steerTarget[Cfg::WHEEL_COUNT];   // steering target, encoder counts (multi-turn)
    int escUs[Cfg::WHEEL_COUNT];         // drive ESC pulse width, us
    int conveyor;                        // signed duty, -MAX..MAX
    int arm;                             // signed duty, -MAX..MAX
    uint8_t valves;                      // bit n = valve on shift-register bit n (0..7)
    bool steerOpenLoop;                  // test only: bypass PID, use steerDuty
    int steerDuty[Cfg::WHEEL_COUNT];
  };

  Command cmd;
  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

  void publish(const Command &c) { portENTER_CRITICAL(&lock); cmd = c; portEXIT_CRITICAL(&lock); }
  Command fetch() { portENTER_CRITICAL(&lock); Command c = cmd; portEXIT_CRITICAL(&lock); return c; }

  Command safeDefault() {
    Command c = {};
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) c.escUs[i] = Cfg::ESC_MIN_US;
    return c;
  }
}

namespace Control {
  struct Pid {
    float p, i, d, lastError, integral;
    float correct(float error, float dt) {
      float diff = (error - lastError) / dt;
      lastError = error;
      integral += error * dt;
      return p * error + d * diff + i * integral;
    }
  };

  Pwm::Out esc[Cfg::WHEEL_COUNT], steer[Cfg::WHEEL_COUNT], conveyor, arm;
  Pid pid[Cfg::WHEEL_COUNT];
  volatile int measured[Cfg::WHEEL_COUNT];   // last encoder reading, for debug prints

  // INA/INB pattern used by both boards: +speed -> A=1,B=0 ; -speed -> A=0,B=1 ; 0 -> A=B=1 (brake)
  inline uint32_t dirBits(int speed, int bitA, int bitB) {
    return ((uint32_t)(speed >= 0) << bitA) | ((uint32_t)(speed <= 0) << bitB);
  }

  inline uint32_t escDuty(int us) {
    us = constrain(us, Cfg::ESC_MIN_US, Cfg::ESC_MAX_US);
    return (uint32_t)((uint64_t)us * ((1u << Cfg::ESC_RES) - 1) * Cfg::ESC_FREQ / 1000000ULL);
  }

  void step(const Shared::Command &c, float dt) {
    uint32_t sr = c.valves;

    // ---- upper motors
    sr |= dirBits(c.conveyor, Cfg::BIT_CONVEYOR_A, Cfg::BIT_CONVEYOR_B);
    sr |= dirBits(c.arm, Cfg::BIT_ARM_A, Cfg::BIT_ARM_B);
    Pwm::write(conveyor, min(abs(c.conveyor), Cfg::MAX_CONVEYOR_DUTY));
    Pwm::write(arm, min(abs(c.arm), Cfg::MAX_ARM_DUTY));

    // ---- drive ESCs
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) Pwm::write(esc[i], escDuty(c.escUs[i]));

    // ---- steering position loop
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      int m = Encoder::count(i);
      measured[i] = m;
      int speed = c.steerOpenLoop ? c.steerDuty[i]
                                  : (int)roundf(pid[i].correct((float)(c.steerTarget[i] - m), dt));
      speed *= Cfg::STEER_MOTOR_DIR[i];
      sr |= dirBits(speed, Cfg::STEER_INA_BITS[i], Cfg::STEER_INB_BITS[i]);
      Pwm::write(steer[i], min(abs(speed), Cfg::STEER_MAX_DUTY));
    }

    Shifter::write(sr);
  }

  void task(void *) {
    const float dt = Cfg::CONTROL_PERIOD_MS * 0.001f;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
      vTaskDelayUntil(&last, pdMS_TO_TICKS(Cfg::CONTROL_PERIOD_MS));
      step(Shared::fetch(), dt);
    }
  }

  void setup() {
    Shared::cmd = Shared::safeDefault();
    Shifter::setup();

    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      esc[i] = Pwm::attach(Cfg::ESC_PINS[i], Cfg::ESC_CH[i], Cfg::ESC_FREQ, Cfg::ESC_RES);
      Pwm::write(esc[i], escDuty(Cfg::ESC_MIN_US));
      steer[i] = Pwm::attach(Cfg::STEER_PWM_PINS[i], Cfg::STEER_CH[i], Cfg::MOTOR_PWM_FREQ, Cfg::MOTOR_PWM_RES);
      Pwm::write(steer[i], 0);
      pid[i] = Pid{Cfg::STEER_P, Cfg::STEER_I, Cfg::STEER_D, 0, 0};
    }
    conveyor = Pwm::attach(Cfg::CONVEYOR_PWM_PIN, Cfg::CONVEYOR_CH, Cfg::MOTOR_PWM_FREQ, Cfg::MOTOR_PWM_RES);
    arm      = Pwm::attach(Cfg::ARM_PWM_PIN, Cfg::ARM_CH, Cfg::MOTOR_PWM_FREQ, Cfg::MOTOR_PWM_RES);
    Pwm::write(conveyor, 0);
    Pwm::write(arm, 0);

    Encoder::setup();

    // Core 1 (Bluetooth runs on core 0), above the Arduino loop (prio 1)
    xTaskCreatePinnedToCore(task, "r1-control", 4096, nullptr, 3, nullptr, 1);
  }
}
