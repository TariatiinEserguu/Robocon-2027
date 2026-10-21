#pragma once
// 4x quadrature encoders on the hardware pulse counter (PCNT).
// Same counting scheme as r1-servo: count both edges of A, B selects direction.
// Supports the legacy driver (IDF 4.4 / Arduino 2.x = Bluepad32 package)
// and the new pulse_cnt driver (IDF 5 / Arduino 3.x).
#include <Arduino.h>
#include <driver/gpio.h>
#include "config.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/pulse_cnt.h>
#else
  #include <driver/pcnt.h>
#endif

namespace Encoder {
  volatile int32_t offsets[Cfg::WHEEL_COUNT] = {};

#if ESP_IDF_VERSION_MAJOR >= 5
  pcnt_unit_handle_t units[Cfg::WHEEL_COUNT] = {};
  int unitIndex[Cfg::WHEEL_COUNT];

  static bool IRAM_ATTR onReach(pcnt_unit_handle_t, const pcnt_watch_event_data_t *e, void *arg) {
    int i = *(int *)arg;
    offsets[i] += e->watch_point_value;          // counter auto-clears at the limit
    return false;
  }

  void setup() {
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      pcnt_unit_config_t ucfg = {};
      ucfg.low_limit  = INT16_MIN;
      ucfg.high_limit = INT16_MAX;
      ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &units[i]));
      ESP_ERROR_CHECK(pcnt_unit_add_watch_point(units[i], INT16_MAX));
      ESP_ERROR_CHECK(pcnt_unit_add_watch_point(units[i], INT16_MIN));

      unitIndex[i] = i;
      pcnt_event_callbacks_t cbs = {};
      cbs.on_reach = onReach;
      ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(units[i], &cbs, &unitIndex[i]));

      pcnt_chan_config_t ccfg = {};
      ccfg.edge_gpio_num  = Cfg::ENC_A_PINS[i];
      ccfg.level_gpio_num = Cfg::ENC_B_PINS[i];
      pcnt_channel_handle_t ch;
      ESP_ERROR_CHECK(pcnt_new_channel(units[i], &ccfg, &ch));
      ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch,
          PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
      ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch,
          PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));   // (high, low)

      gpio_set_pull_mode((gpio_num_t)Cfg::ENC_A_PINS[i], GPIO_FLOATING);
      gpio_set_pull_mode((gpio_num_t)Cfg::ENC_B_PINS[i], GPIO_FLOATING);

      ESP_ERROR_CHECK(pcnt_unit_enable(units[i]));
      ESP_ERROR_CHECK(pcnt_unit_clear_count(units[i]));
      ESP_ERROR_CHECK(pcnt_unit_start(units[i]));
    }
  }

  static int rawCount(int i) {
    int c = 0;
    pcnt_unit_get_count(units[i], &c);
    return c;
  }

#else  // ---------------- legacy driver (Bluepad32 / Arduino 2.x) ----------------

  static void IRAM_ATTR onLimit(void *arg) {
    int i = (int)(intptr_t)arg;
    uint32_t status = 0;
    pcnt_get_event_status((pcnt_unit_t)i, &status);
    if (status & PCNT_EVT_H_LIM) offsets[i] += INT16_MAX;   // counter resets to 0 at the limit
    if (status & PCNT_EVT_L_LIM) offsets[i] += INT16_MIN;
  }

  void setup() {
    ESP_ERROR_CHECK(pcnt_isr_service_install(0));
    for (int i = 0; i < Cfg::WHEEL_COUNT; i++) {
      pcnt_unit_t unit = (pcnt_unit_t)i;
      pcnt_config_t c = {};
      c.pulse_gpio_num = Cfg::ENC_A_PINS[i];
      c.ctrl_gpio_num  = Cfg::ENC_B_PINS[i];
      c.channel        = PCNT_CHANNEL_0;
      c.unit           = unit;
      c.pos_mode       = PCNT_COUNT_INC;     // A rising
      c.neg_mode       = PCNT_COUNT_DEC;     // A falling
      c.hctrl_mode     = PCNT_MODE_REVERSE;  // B high -> invert
      c.lctrl_mode     = PCNT_MODE_KEEP;     // B low  -> keep
      c.counter_h_lim  = INT16_MAX;
      c.counter_l_lim  = INT16_MIN;
      ESP_ERROR_CHECK(pcnt_unit_config(&c));

      // legacy driver turns on pull-ups; r1-servo used floating inputs
      gpio_set_pull_mode((gpio_num_t)Cfg::ENC_A_PINS[i], GPIO_FLOATING);
      gpio_set_pull_mode((gpio_num_t)Cfg::ENC_B_PINS[i], GPIO_FLOATING);

      pcnt_event_enable(unit, PCNT_EVT_H_LIM);
      pcnt_event_enable(unit, PCNT_EVT_L_LIM);
      pcnt_counter_pause(unit);
      pcnt_counter_clear(unit);
      ESP_ERROR_CHECK(pcnt_isr_handler_add(unit, onLimit, (void *)(intptr_t)i));
      pcnt_intr_enable(unit);
      pcnt_counter_resume(unit);
    }
  }

  static int rawCount(int i) {
    int16_t c = 0;
    pcnt_get_counter_value((pcnt_unit_t)i, &c);
    return c;
  }
#endif

  // Multi-turn position in counts (overflow-safe read)
  int count(int i) {
    int32_t off, c;
    do {
      off = offsets[i];
      c = rawCount(i);
    } while (off != offsets[i]);
    return (int)(off + c) * Cfg::ENC_DIR[i];
  }
}
