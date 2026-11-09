# IST8310 magnetometer — CubeMX + driver (RoboMaster Type-C)

> Branch: `feature/ist8310-mag` (from `feature/bsp-board-io`)  
> Status: **driver landed** in `UserLib/IST8310`; imuTask SMOKE prints `merr/mok/mx/my/mz`.  
> I2C3 (PA8/PC9, Fast 400 kHz + DMA) is merged into this branch from local CubeMX.  
> **PG6 RSTN / PG3 DRDY are NOT in `.ioc`** — soft-initialized inside the driver.

## Hardware

| Item | Value |
|---|---|
| Bus | **I2C3** SCL=**PA8**, SDA=**PC9**, Fast 400 kHz |
| HAL DevAddress | **`0x1C`** (`0x0E << 1`) |
| 7-bit address | **`0x0E`** |
| WHO_AM_I | reg `0x00` = `0x10` |
| RSTN | **PG6**, active-low pulse (soft GPIO) |
| DRDY | **PG3**, input (soft GPIO; v1 polls data regs) |

Do **not** put the mag on I2C1 (servo/PCA9685).

## API (`UserLib/IST8310`)

```c
void IST8310_StructInit(ist8310_t *dev);
ist8310_err_t IST8310_Init(ist8310_t *dev);   /* soft GPIO + RSTN pulse + WhoAmI + cont. 200 Hz */
uint8_t IST8310_WhoAmI(void);                 /* 0x10 ok, 0xFF on I2C fail */
ist8310_err_t IST8310_Read(ist8310_t *dev);   /* raw_mx/my/mz + mag_ut[] */
```

Init sequence (RoboMaster-style): RSTN low 50 ms → high 50 ms → check WAI → `AVGCNTL=0x09`, `PDCNTL=0xC0`, `CNTL1=0x0B`.

## Smoke UART

Same path as BMI088: USART6 @ **9600 8N1** via onboard BT.

```text
SMOKE ready USART6 9600 8N1 (key/mode/err/ok + imu milli + mag raw)
SMOKE key=0 mode=0 led=HB err=0 ok=100 ax=... merr=0 mok=100 mx=12 my=-34 mz=981
```

| Field | Meaning |
|---|---|
| `merr` | `g_ist8310_last_err` (`0` = OK) |
| `mok` | successful mag reads (should climb) |
| `mx my mz` | raw LSB (`ist8310_t.raw_m*`) |

Mag init is best-effort: BMI088 smoke still runs if mag fails; imuTask retries mag ~1 Hz.

## Optional later CubeMX

Add User Labels `RSTN_IST8310` (PG6 out high) and `DRDY_IST8310` (PG3 in); then driver local `#ifndef` pin macros can switch to `main.h`. EXTI on DRDY is optional.

## Non-goals

- No Bridge / yaw fusion on this PR  
- No force-push / merge to main  
- Do not commit I2C3 onto `feature/usart-module-port` (copy only)