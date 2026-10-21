#pragma once
// Thin LEDC wrapper that works on Arduino-ESP32 2.x (Bluepad32 package) and 3.x.
#include <Arduino.h>

namespace Pwm {
  struct Out { int pin; int ch; };

  inline Out attach(int pin, int ch, int freq, int res) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin, freq, res, ch);
#else
    ledcSetup(ch, freq, res);
    ledcAttachPin(pin, ch);
#endif
    return Out{pin, ch};
  }

  inline void write(const Out &o, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(o.pin, duty);
#else
    ledcWrite(o.ch, duty);
#endif
  }
}
