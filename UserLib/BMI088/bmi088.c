/**
 * @file bmi088.c
 * @brief Polling SPI BMI088 bring-up + sample read (QGimbal logic, C port).
 *
 * SPI1: SCK=PB3, MISO=PB4, MOSI=PA7
 * CS:   CS1_ACCEL=PA4, CS1_GYRO=PB0
 * Mode: CPOL=1 CPHA=1 (SPI Mode 3), soft NSS
 */
#include "bmi088.h"
#include "bmi088_reg.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f
#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_US 150
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_GYRO_2000_SEN 0.0010652644360316953f

#define GYRO_CAL_WARMUP_SAMPLES 20u
/** Per-axis deviation from mean above this (rad/s) counts as outlier. */
#define GYRO_CAL_MOTION_RAD_S 0.025f
/** Minimum % of samples that must agree with trimmed mean. */
#define GYRO_CAL_MIN_INLIER_PCT 85u

static float s_accel_sen = BMI088_ACCEL_3G_SEN;
static float s_gyro_sen = BMI088_GYRO_2000_SEN;

static void delay_ms(uint16_t ms)
{
  osDelay(ms);
}

static void delay_us_approx(uint16_t us)
{
  /* FreeRTOS tick is coarse; pad to at least 1 ms for sensor settle waits. */
  (void)us;
  osDelay(1);
}

static void accel_ns_l(void)
{
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

static void accel_ns_h(void)
{
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

static void gyro_ns_l(void)
{
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

static void gyro_ns_h(void)
{
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

static uint8_t rw_byte(uint8_t tx)
{
  uint8_t rx = 0;
  (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
  return rx;
}

static void write_single_reg(uint8_t reg, uint8_t data)
{
  (void)rw_byte(reg);
  (void)rw_byte(data);
}

static void read_single_reg(uint8_t reg, uint8_t *data)
{
  (void)rw_byte((uint8_t)(reg | 0x80u));
  *data = rw_byte(0x55);
}

static void read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  (void)rw_byte((uint8_t)(reg | 0x80u));
  while (len != 0u) {
    *buf = rw_byte(0x55);
    buf++;
    len--;
  }
}

static void accel_write_single_reg(uint8_t reg, uint8_t data)
{
  accel_ns_l();
  write_single_reg(reg, data);
  accel_ns_h();
}

static void accel_read_single_reg(uint8_t reg, uint8_t *data)
{
  accel_ns_l();
  (void)rw_byte((uint8_t)(reg | 0x80u));
  (void)rw_byte(0x55); /* accel SPI dummy */
  *data = rw_byte(0x55);
  accel_ns_h();
}

static void accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  accel_ns_l();
  (void)rw_byte((uint8_t)(reg | 0x80u));
  read_muli_reg(reg, data, len);
  accel_ns_h();
}

static void gyro_write_single_reg(uint8_t reg, uint8_t data)
{
  gyro_ns_l();
  write_single_reg(reg, data);
  gyro_ns_h();
}

static void gyro_read_single_reg(uint8_t reg, uint8_t *data)
{
  gyro_ns_l();
  read_single_reg(reg, data);
  gyro_ns_h();
}

static void gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  gyro_ns_l();
  read_muli_reg(reg, data, len);
  gyro_ns_h();
}

static bmi088_err_t accel_init(void)
{
  uint8_t res = 0;
  static const uint8_t cfg[][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ERR_ACC_PWR_CTRL},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ERR_ACC_PWR_CONF},
    {BMI088_ACC_CONF, (uint8_t)(BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set), BMI088_ERR_ACC_CONF},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ERR_ACC_RANGE},
  };

  accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);
  accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);

  accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  delay_ms(BMI088_LONG_DELAY_TIME);

  accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);
  accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);

  if (res != BMI088_ACC_CHIP_ID_VALUE) {
    return BMI088_ERR_NO_SENSOR;
  }

  for (unsigned i = 0; i < (sizeof(cfg) / sizeof(cfg[0])); i++) {
    accel_write_single_reg(cfg[i][0], cfg[i][1]);
    delay_us_approx(BMI088_COM_WAIT_US);
    accel_read_single_reg(cfg[i][0], &res);
    delay_us_approx(BMI088_COM_WAIT_US);
    if (res != cfg[i][1]) {
      return (bmi088_err_t)cfg[i][2];
    }
  }
  return BMI088_OK;
}

static bmi088_err_t gyro_init(void)
{
  uint8_t res = 0;
  static const uint8_t cfg[][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_ERR_GYRO_RANGE},
    {BMI088_GYRO_BANDWIDTH, (uint8_t)(BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_ERR_GYRO_BANDWIDTH},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_ERR_GYRO_LPM1},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_ERR_GYRO_CTRL},
  };

  gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  delay_ms(BMI088_LONG_DELAY_TIME);

  gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);
  gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  delay_us_approx(BMI088_COM_WAIT_US);

  if (res != BMI088_GYRO_CHIP_ID_VALUE) {
    return BMI088_ERR_NO_SENSOR;
  }

  for (unsigned i = 0; i < (sizeof(cfg) / sizeof(cfg[0])); i++) {
    gyro_write_single_reg(cfg[i][0], cfg[i][1]);
    delay_us_approx(BMI088_COM_WAIT_US);
    gyro_read_single_reg(cfg[i][0], &res);
    delay_us_approx(BMI088_COM_WAIT_US);
    if (res != cfg[i][1]) {
      return (bmi088_err_t)cfg[i][2];
    }
  }
  return BMI088_OK;
}

void BMI088_StructInit(bmi088_t *dev)
{
  if (dev == NULL) {
    return;
  }
  dev->initialized = false;
  dev->accel[0] = dev->accel[1] = dev->accel[2] = 0.0f;
  dev->gyro[0] = dev->gyro[1] = dev->gyro[2] = 0.0f;
  dev->gyro_bias[0] = dev->gyro_bias[1] = dev->gyro_bias[2] = 0.0f;
  dev->gyro_calibrated = false;
  dev->temperature_c = 0.0f;
}

bmi088_err_t BMI088_Init(bmi088_t *dev)
{
  bmi088_err_t err;

  if (dev == NULL) {
    return BMI088_ERR_NO_SENSOR;
  }

  accel_ns_h();
  gyro_ns_h();

  err = accel_init();
  if (err != BMI088_OK) {
    dev->initialized = false;
    return err;
  }
  err = gyro_init();
  if (err != BMI088_OK) {
    dev->initialized = false;
    return err;
  }

  s_accel_sen = BMI088_ACCEL_3G_SEN;
  s_gyro_sen = BMI088_GYRO_2000_SEN;
  dev->initialized = true;
  return BMI088_OK;
}

void BMI088_Read(bmi088_t *dev)
{
  uint8_t buf[6];
  uint8_t tbuf[2];
  int16_t traw;

  if (dev == NULL || !dev->initialized) {
    return;
  }

  accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
  dev->accel[0] = ((int16_t)((buf[1] << 8) | buf[0])) * s_accel_sen;
  dev->accel[1] = ((int16_t)((buf[3] << 8) | buf[2])) * s_accel_sen;
  dev->accel[2] = ((int16_t)((buf[5] << 8) | buf[4])) * s_accel_sen;

  gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
  dev->gyro[0] = ((int16_t)((buf[1] << 8) | buf[0])) * s_gyro_sen - dev->gyro_bias[0];
  dev->gyro[1] = ((int16_t)((buf[3] << 8) | buf[2])) * s_gyro_sen - dev->gyro_bias[1];
  dev->gyro[2] = ((int16_t)((buf[5] << 8) | buf[4])) * s_gyro_sen - dev->gyro_bias[2];

  accel_read_muli_reg(BMI088_TEMP_M, tbuf, 2);
  traw = (int16_t)((tbuf[0] << 3) | (tbuf[1] >> 5));
  if (traw > 1023) {
    traw -= 2048;
  }
  dev->temperature_c = (float)traw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


static void gyro_sample_rad(uint8_t buf[6], float out[3])
{
  out[0] = ((int16_t)((buf[1] << 8) | buf[0])) * s_gyro_sen;
  out[1] = ((int16_t)((buf[3] << 8) | buf[2])) * s_gyro_sen;
  out[2] = ((int16_t)((buf[5] << 8) | buf[4])) * s_gyro_sen;
}

static bool gyro_sample_is_inlier(const float g[3], const float mean[3])
{
  float d0 = g[0] - mean[0];
  float d1 = g[1] - mean[1];
  float d2 = g[2] - mean[2];
  if (d0 < 0.0f) {
    d0 = -d0;
  }
  if (d1 < 0.0f) {
    d1 = -d1;
  }
  if (d2 < 0.0f) {
    d2 = -d2;
  }
  return (d0 <= GYRO_CAL_MOTION_RAD_S) && (d1 <= GYRO_CAL_MOTION_RAD_S) &&
         (d2 <= GYRO_CAL_MOTION_RAD_S);
}

bmi088_err_t BMI088_CalibrateGyro(bmi088_t *dev, uint16_t samples, uint16_t delay_ms)
{
  float sum[3] = {0.0f, 0.0f, 0.0f};
  float mean[3];
  float inlier_sum[3] = {0.0f, 0.0f, 0.0f};
  uint16_t i;
  uint16_t inliers = 0u;
  uint8_t buf[6];
  float g[3];
  uint16_t min_inliers;

  if (dev == NULL || !dev->initialized) {
    return BMI088_ERR_NO_SENSOR;
  }
  if (samples < 10u) {
    samples = 10u;
  }
  if (delay_ms < 1u) {
    delay_ms = 1u;
  }

  dev->gyro_bias[0] = dev->gyro_bias[1] = dev->gyro_bias[2] = 0.0f;
  dev->gyro_calibrated = false;

  for (i = 0u; i < GYRO_CAL_WARMUP_SAMPLES; ++i) {
    gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
    osDelay(delay_ms);
  }

  for (i = 0u; i < samples; ++i) {
    gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
    gyro_sample_rad(buf, g);
    sum[0] += g[0];
    sum[1] += g[1];
    sum[2] += g[2];
    osDelay(delay_ms);
  }

  mean[0] = sum[0] / (float)samples;
  mean[1] = sum[1] / (float)samples;
  mean[2] = sum[2] / (float)samples;

  for (i = 0u; i < samples; ++i) {
    gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
    gyro_sample_rad(buf, g);
    if (gyro_sample_is_inlier(g, mean)) {
      inlier_sum[0] += g[0];
      inlier_sum[1] += g[1];
      inlier_sum[2] += g[2];
      inliers++;
    }
    osDelay(delay_ms);
  }

  min_inliers = (uint16_t)(((uint32_t)samples * (uint32_t)GYRO_CAL_MIN_INLIER_PCT) / 100u);
  if (min_inliers < 10u) {
    min_inliers = 10u;
  }
  if (inliers < min_inliers) {
    return BMI088_ERR_GYRO_MOTION;
  }

  dev->gyro_bias[0] = inlier_sum[0] / (float)inliers;
  dev->gyro_bias[1] = inlier_sum[1] / (float)inliers;
  dev->gyro_bias[2] = inlier_sum[2] / (float)inliers;
  dev->gyro_calibrated = true;
  return BMI088_OK;
}
