#pragma once
// 3x 74HC595 daisy chain (U1 -> U2 -> U3), 24 outputs.
// Replaces: the 16-bit chain on r1-controller (CS=GPIO5) and the
//           8-bit register on r1-servo (CS=GPIO19).
// Only the control task writes to it, so no locking is needed here.
#include <SPI.h>
#include "config.h"

namespace Shifter {
  SPIClass *spi = nullptr;
  uint32_t last = 0xFFFFFFFF;   // force first write

  void write(uint32_t data) {
    data &= 0xFFFFFF;
    if (data == last) return;
    last = data;

    spi->beginTransaction(SPISettings(Cfg::SR_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(Cfg::SR_LATCH, LOW);
    // MSB first: the last byte shifted ends up in U1 (closest to the ESP32)
    spi->transfer((data >> 16) & 0xFF);   // -> U3 (steering)
    spi->transfer((data >> 8) & 0xFF);    // -> U2 (upper motors)
    spi->transfer(data & 0xFF);           // -> U1 (valves)
    digitalWrite(Cfg::SR_LATCH, HIGH);    // rising edge latches outputs
    spi->endTransaction();
  }

  void setup() {
    if (Cfg::SR_OE >= 0) {               // keep outputs off until a known state is latched
      pinMode(Cfg::SR_OE, OUTPUT);
      digitalWrite(Cfg::SR_OE, HIGH);
    }
    pinMode(Cfg::SR_LATCH, OUTPUT);
    digitalWrite(Cfg::SR_LATCH, HIGH);

    spi = new SPIClass(HSPI);
    spi->begin(Cfg::SR_SCLK, -1, Cfg::SR_MOSI, -1);

    write(0);                            // all valves closed, all H-bridges off
    if (Cfg::SR_OE >= 0) digitalWrite(Cfg::SR_OE, LOW);   // enable outputs
  }
}
