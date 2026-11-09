# Board IO (DJI RoboMaster Type-C)

Pin map verified against `CubeMx/docs/RoboMaster 开发板 C 型用户手册.pdf` and QGimbal `Core/Inc/main.h`.

| Function | Net | GPIO | Notes |
|---|---|---|---|
| LED_R | LED_R | PH12 | High = on |
| LED_G | LED_G | PH11 | High = on |
| LED_B | LED_B | PH10 | High = on |
| KEY | KEY | PA0 | Pressed = low (10k pull-up) |
| CS1_ACCEL | CS1_Accel | PA4 | Soft NSS, idle high |
| CS1_GYRO | CS1_Gyro | PB0 | Soft NSS, idle high |
| SPI1_SCK | SPI1_CLK | PB3 | AF5 |
| SPI1_MISO | SPI1_MISO | PB4 | AF5 |
| SPI1_MOSI | SPI1_MOSI | PA7 | AF5 |

## Layout (QGimbal-style)

- `BSP/Board` — LED / KEY / CS GPIO helpers
- `UserLib/BMI088` — polling SPI driver (C port of QGimbal `UserLib/BMI088`)
- `Applications` — `boardTask` (heartbeat + key), `imuTask` (init + 100 Hz read + smoke UART)

## CubeMX / DMA caveats

- `Core/Src/spi.c` is **hand-maintained** (polling, Mode 3). Opening CubeMX and enabling SPI1 is recommended later so `.ioc` matches; until then keep USER CODE and this file.
- USART1/6 and I2C1 DMA streams are **unchanged** on this branch.
- Free for future SPI DMA (same as QGimbal): `DMA2_Stream0` (RX) + `DMA2_Stream3` (TX), channel 3 — do **not** steal USART/I2C streams.

## Smoke test (LED / KEY / debugger)

1. Build & flash `build/Debug/carry_bot_v1.elf`.
2. **LED**: green should blink ~1 Hz after boot.
3. **KEY**: hold KEY → red solid; click KEY cycles heartbeat → R → G → B → …
4. **BMI088**: in debugger watch `g_bmi088_last_err` (0 = OK), `g_bmi088_ok_count` increasing, `g_bmi088.accel/gyro/temperature_c` changing when board is moved. Init fail → blue LED toggles while retrying.

## Smoke UART (SerialTest)

Path: **USART6** (PG14 TX / PG9 RX) → onboard BT module → phone/PC SPP.

| Setting | Value |
|---|---|
| Tool | SerialTest (Bluetooth SPP) or any serial terminal on the BT virtual COM |
| Baud | **9600** 8N1 (BT DEBUG / `BT_DEFAULT_BAUDRATE`) |
| Peer | Pair the board BT module in Windows, then open SPP in SerialTest |

After BMI088 init OK, one banner then ~1 Hz lines:

```text
SMOKE ready USART6 9600 8N1 (key/mode/err/ok + imu milli)
SMOKE key=0 mode=0 led=HB err=0 ok=100 ax=12 ay=-34 az=9810 gx=1 gy=-2 gz=0 t=245
```

Field meanings (stable for eyeballing):

| Field | Meaning |
|---|---|
| `key` | `1` while KEY held, else `0` |
| `mode` | `0` heartbeat, `1` R, `2` G, `3` B (KEY click advances) |
| `led` | `RED` if key held, else `HB`/`R`/`G`/`B` |
| `err` | `g_bmi088_last_err` (`0` = OK) |
| `ok` | `g_bmi088_ok_count` (must climb ~100 / s) |
| `ax ay az` | accel ×1000 (milli units of driver scale) |
| `gx gy gz` | gyro ×1000 (milli rad/s) |
| `t` | temperature ×10 (°C decidegrees), e.g. `245` ≈ 24.5 °C |

Expect: idle `key=0 led=HB err=0` and rising `ok`; hold KEY → `key=1 led=RED`; click → `mode`/`led` cycle; tilt board → `ax/ay/az` move.