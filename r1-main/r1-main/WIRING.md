# R1 – single ESP32 wiring

One ESP32-DevKitC (WROOM-32) replaces `r1-controller` + `r1-servo`.
The I2C link between the boards is removed. Schematic: `r1-wiring.svg` / `r1-wiring.png`.

## Pin map

| GPIO    | Net         | Connects to                          | Old board   |
|---------|-------------|--------------------------------------|-------------|
| 4       | ESC0        | ESC 0 signal                         | ctrl 32     |
| 13      | ESC1        | ESC 1 signal                         | ctrl 33     |
| 19      | ESC2        | ESC 2 signal                         | ctrl 25     |
| 21      | ESC3        | ESC 3 signal                         | ctrl 26     |
| 17      | STEER0_PWM  | steering H-bridge 0 PWM              | servo 17    |
| 16      | STEER1_PWM  | steering H-bridge 1 PWM              | servo 16    |
| 27      | STEER2_PWM  | steering H-bridge 2 PWM              | servo 27    |
| 14      | STEER3_PWM  | steering H-bridge 3 PWM              | servo 14    |
| 39 / 36 | ENC0 A / B  | encoder module 0                     | servo 39/36 |
| 35 / 34 | ENC1 A / B  | encoder module 1                     | servo 35/34 |
| 33 / 32 | ENC2 A / B  | encoder module 2                     | servo 33/32 |
| 26 / 25 | ENC3 A / B  | encoder module 3                     | servo 26/25 |
| 22      | CONV_PWM    | conveyor H-bridge PWM                | ctrl 27     |
| 2       | ARM_PWM     | arm H-bridge PWM                     | ctrl 14     |
| 23      | SR_DATA     | U1 DS (pin 14)                       | ctrl 23     |
| 18      | SR_CLK      | SHCP (pin 11) of U1, U2, U3          | ctrl 18     |
| 5       | SR_LATCH    | STCP (pin 12) of U1, U2, U3          | ctrl 5      |
| 0       | SR_OE       | /OE (pin 13) of U1–U3 + 10k to 3V3   | new         |

Unused on purpose: 12, 15 (strapping), 1/3 (USB serial), 6–11 (flash).

## 74HC595 chain (24 bits)

ESP32 GPIO23 → U1 DS · U1 Q7' (9) → U2 DS (14) · U2 Q7' (9) → U3 DS (14).
U1+U2 = old controller shift board, U3 = old servo-board 595.

| Bits  | Chip | Function                                                               |
|-------|------|------------------------------------------------------------------------|
| 0–6   | U1   | valves: RAISE, EXTEND, GRAB, W-LIFT, W-TILT, W-GRAB1, W-GRAB2 (7 spare)|
| 8–11  | U2   | conveyor INA/INB, arm INA/INB (12–15 spare)                            |
| 16–23 | U3   | S0 INA/INB, S1 INA/INB, S3 INA/INB, S2 INA/INB                         |

## Rewiring checklist

1. Remove the r1-servo ESP32 and the I2C wires.
2. Steering wires (encoders + steering PWM) go to the same GPIO numbers on the main ESP32.
3. ESC signals: 32/33/25/26 → 4/13/19/21.
4. Conveyor PWM 27 → 22, arm PWM 14 → 2.
5. Chain U3 after U2; U3 SHCP → 18, STCP → 5 (was CS 19).
6. All /OE pins → GPIO0 + 10 kΩ pull-up to 3V3. /MR → 3V3.
7. One common ground everywhere.

## Notes

- WROOM only (WROVER uses GPIO16/17 for PSRAM).
- ESP32 inputs are 3.3 V max: encoders must be 3.3 V or level-shifted.
- Nothing may pull GPIO2 high at boot, or flashing fails.
- 30-pin DevKit has no GPIO0 on the header: tie /OE to GND, set `SR_OE = -1` in `config.h`.
