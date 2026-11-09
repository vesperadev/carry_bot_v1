/**
 * @file ist8310.h
 * @brief IST8310 magnetometer driver (C), I2C3 polling + soft RSTN/DRDY GPIO.
 */
#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  IST8310_OK = 0x00,
  IST8310_ERR_WHOAMI = 0x01,
  IST8310_ERR_COMM = 0x02,
  IST8310_ERR_NO_SENSOR = 0xFF,
} ist8310_err_t;

typedef struct {
  bool initialized;
  /** True after N consecutive reads with sane field strength (see driver). */
  bool mag_ready;
  int16_t raw_mx;
  int16_t raw_my;
  int16_t raw_mz;
  float mag_ut[3]; /* microtesla */
} ist8310_t;

void IST8310_StructInit(ist8310_t *dev);
ist8310_err_t IST8310_Init(ist8310_t *dev);
/** Read WHO_AM_I (expects 0x10). Returns register value, or 0xFF on I2C fail. */
uint8_t IST8310_WhoAmI(void);
/** PG3 DRDY line high (new sample). Poll-friendly. */
bool IST8310_DataReadyPin(void);
/** Block up to timeout_ms for DRDY; returns false if still not ready. */
bool IST8310_WaitDataReady(uint32_t timeout_ms);
/** Fill raw_mx/my/mz only (no scale). */
ist8310_err_t IST8310_ReadRaw(ist8310_t *dev);
/** Read raw XYZ into raw_m* and scaled mag_ut[]. */
ist8310_err_t IST8310_Read(ist8310_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* IST8310_H */
