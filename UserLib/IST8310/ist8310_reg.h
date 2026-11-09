/**
 * @file ist8310_reg.h
 * @brief IST8310 register map (RoboMaster Type-C onboard mag, I2C3).
 */
#ifndef IST8310_REG_H
#define IST8310_REG_H

/** 7-bit slave address (datasheet / LA). */
#define IST8310_I2C_ADDR_7BIT 0x0Eu
/** HAL DevAddress = 7-bit << 1. */
#define IST8310_I2C_ADDR_HAL (IST8310_I2C_ADDR_7BIT << 1) /* 0x1C */

#define IST8310_REG_WAI 0x00u
#define IST8310_WAI_VALUE 0x10u

#define IST8310_REG_STAT1 0x02u
#define IST8310_REG_DATAXL 0x03u
#define IST8310_REG_DATAXH 0x04u
#define IST8310_REG_DATAYL 0x05u
#define IST8310_REG_DATAYH 0x06u
#define IST8310_REG_DATAZL 0x07u
#define IST8310_REG_DATAZH 0x08u
#define IST8310_REG_STAT2 0x09u

#define IST8310_REG_CNTL1 0x0Au
#define IST8310_REG_CNTL2 0x0Bu
#define IST8310_REG_STR 0x0Cu
#define IST8310_REG_CNTL3 0x0Du

#define IST8310_REG_TEMPL 0x1Cu
#define IST8310_REG_TEMPH 0x1Du

#define IST8310_REG_AVGCNTL 0x41u
#define IST8310_REG_PDCNTL 0x42u

/** Continuous measurement ~200 Hz (RoboMaster C-board bring-up). */
#define IST8310_CNTL1_CONT_200HZ 0x0Bu
#define IST8310_AVGCNTL_VALUE 0x09u
#define IST8310_PDCNTL_VALUE 0xC0u

/** Sensitivity: 0.3 uT / LSB. */
#define IST8310_MAG_SEN 0.3f

#endif /* IST8310_REG_H */
