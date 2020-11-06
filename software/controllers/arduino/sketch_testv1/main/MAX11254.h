/*
MAX11254.h - MAX11254 ADC functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef MAX11254_h
#define MAX11254_h

#include <SPI.h>
#include "Globals.h"

// Register map (page 38)
#define MAX11254_REG_STAT                0
#define MAX11254_REG_CTRL1               1
#define MAX11254_REG_CTRL2               2
#define MAX11254_REG_CTRL3               3
#define MAX11254_REG_GPIO_CTRL           4
#define MAX11254_REG_DELAY               5
#define MAX11254_REG_CHMAP1              6
#define MAX11254_REG_CHMAP0              7
#define MAX11254_REG_SEQ                 8
#define MAX11254_REG_GPO_DIR             9
#define MAX11254_REG_SOC                10
#define MAX11254_REG_SGC                11
#define MAX11254_REG_SCOC               12
#define MAX11254_REG_SCGC               13
#define MAX11254_REG_DATA0              14
#define MAX11254_REG_DATA1              15
#define MAX11254_REG_DATA2              16
#define MAX11254_REG_DATA3              17
#define MAX11254_REG_DATA4              18
#define MAX11254_REG_DATA5              19


/* MAX11254_REG_STAT                            */
#define MAX11254_STAT_DEFAULT               0x00_00_00  //READONLY
#define MAX11254_STAT_INRESET_FILTER(x)     ((BIT(22) & (x)) >> 22)
#define MAX11254_STAT_SRDY_FILTER(x)        ((GENMASK(21, 16) & (x)) >> 16)
#define MAX11254_STAT_SCANERR_FILTER(x)     ((BIT(15) & (x)) >> 15)
#define MAX11254_STAT_REFDET_FILTER(x)      ((BIT(14) & (x)) >> 14)
#define MAX11254_STAT_ORDERR_FILTER(x)      ((BIT(13) & (x)) >> 13)
#define MAX11254_STAT_GPOERR_FILTER(x)      ((BIT(12) & (x)) >> 12)
#define MAX11254_STAT_ERROR_FILTER(x)       ((BIT(11) & (x)) >> 11)
#define MAX11254_STAT_SYSGOR_FILTER(x)      ((BIT(10) & (x)) >> 10)
#define MAX11254_STAT_DOR_FILTER(x)         ((BIT(9) & (x)) >> 9)
#define MAX11254_STAT_AOR_FILTER(x)         ((BIT(8) & (x)) >> 8)
#define MAX11254_STAT_RATE_FILTER(x)        ((GENMASK(7, 4) & (x)) >> 4)
#define MAX11254_STAT_PDSTAT_FILTER(x)      ((GENMASK(3, 2) & (x)) >> 2)
#define MAX11254_STAT_MSTAT_FILTER(x)       ((BIT(1) & (x)) >> 1)
#define MAX11254_STAT_RDY_FILTER(x)         ((BIT(0) & (x)) >> 0)

/* MAX11254_REG_CTRL1                           */
#define MAX11254_CTRL1_DEFAULT              0x26
// STANDBY on powerdown; bipolar range with offset binary format.
#define MAX11254_CTRL1_CAL_MASK             GENMASK(7, 6)
#define MAX11254_CTRL1_CAL_MODE(x)          (((x) & 0x3) << 6)
#define MAX11254_CTRL1_PD_MASK              GENMASK(5, 4)
#define MAX11254_CTRL1_PD_MODE(x)           (((x) & 0x3) << 4)
#define MAX11254_CTRL1_U_B_MASK             BIT(3)
#define MAX11254_CTRL1_U_B_MODE(x)          (((x) & 0x1) << 3)

/* MAX11254_REG_CTRL2                           */
#define MAX11254_CTRL2_DEFAULT              0x20
// Manual default (no fault detection on sensors)
#define MAX11254_CTRL2_PGAEN_MASK           BIT(3)  // Default disabled
#define MAX11254_CTRL2_PGAEN_MODE(x)        (((x) & 0x1) << 3)
#define MAX11254_CTRL2_PGA_MASK             GENMASK(2, 0)   // Default 0 (gain=1)
#define MAX11254_CTRL2_PGA_MODE(x)          (((x) & 0x7) << 0)

/* MAX11254_REG_CTRL3                           */
#define MAX11254_CTRL3_DEFAULT              0x10    // Both system and self- calibration enabled

/* MAX11254_REG_GPIO_CTRL                       */
#define MAX11254_GPIO_CTRL_DEFAULT          0xC0    // Both GPIO enabled as inputs

/* MAX11254_REG_DELAY                           */
#define MAX11254_DELAY_DEFAULT              0x0000  // No delays in the ADC

/* MAX11254_REG_CHMAP1                          */

/* MAX11254_REG_CHMAP0                          */

/* MAX11254_REG_SEQ                             */
#define MAX11254_SEQ_DEFAULT                0x20    // (MUX=1)
#define MAX11254_SEQ_MUX_MSK                GENMASK(7, 5)
#define MAX11254_SEQ_MUX_MODE(x)            (((x) & 0x7) << 5)
#define MAX11254_SEQ_MODE_MSK               GENMASK(4, 3)
#define MAX11254_SEQ_MODE_MODE(x)           (((x) & 0x3) << 3)


/* MAX11254_REG_GPO_DIR                         */

/* MAX11254_REG_SOC                             */

/* MAX11254_REG_SGC                             */

/* MAX11254_REG_SCOC                            */

/* MAX11254_REG_SCGC                            */

/* MAX11254_REG_DATA0                           */

/* MAX11254_REG_DATA1                           */

/* MAX11254_REG_DATA2                           */

/* MAX11254_REG_DATA3                           */

/* MAX11254_REG_DATA4                           */

/* MAX11254_REG_DATA5                           */


/* MAX11254 Command Bytes                       */
#define MAX11254_POWERDOWN              0x90
#define MAX11254_CALIBRATE              0xA0
#define MAX11254_CONVERT(rate)          ((0xB0) | (rate & 0xF))
#define MAX11254_REG_READ(addr)         ((0xC0) | ((addr & 0x1F) << 1) | 0x1)
#define MAX11254_REG_WRITE(addr)        ((0xC0) | ((addr & 0x1F) << 1) | 0x0)

/* MAX11254 Other constants                     */

#define MAX11254_SPI_CLOCK                1000000
#define MAX11254_CONVERSION_TIMEOUT_MS    1000

/* MAX11254 Low-level functions                 */

exit_code max11254_reg_write (
    spi_device dev, uint8_t reg_address, uint8_t* reg_value, uint8_t reg_size);
exit_code max11254_reg_read (
    spi_device dev, uint8_t reg_address, uint8_t* reg_value, uint8_t reg_size);
exit_code max11254_reg_read8 (spi_device dev, uint8_t reg_address, uint8_t* reg_value);
exit_code max11254_reg_read24 (spi_device dev, uint8_t reg_address, uint32_t* reg_value);
exit_code max11254_reg_read_status (spi_device dev, uint32_t* reg_value);
exit_code max11254_reg_write8 (spi_device dev, uint8_t reg_address, uint8_t* reg_value);
exit_code max11254_reg_write8_mask (
    spi_device dev, uint8_t reg_address, uint8_t reg_value, uint8_t mask);
exit_code max11254_command (spi_device dev, uint8_t cmd, bool fast);
exit_code max11254_powerdown (spi_device dev);
exit_code max11254_calibrate (spi_device dev, uint8_t type);
exit_code max11254_swreset (spi_device dev);
exit_code max11254_setup (spi_device dev, bool skip_reset = false);
exit_code max11254_prepare_convert (spi_device dev, uint8_t pga);
exit_code max11254_convert (spi_device dev, uint8_t channel, uint8_t rate, uint32_t* data);

#endif
