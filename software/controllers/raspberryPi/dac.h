/*
dac.h - AD5753 DAC functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#ifndef DAC_h
#define DAC_h

#include "globals.h"

// REUSE code from https://github.com/analogdevicesinc/no-OS/tree/master/ad5758-sdz

/* AD5753 registers definition */
#define DAC_REG_NOP                       0x00
#define DAC_REG_DAC_INPUT                 0x01
#define DAC_REG_DAC_OUTPUT                0x02
#define DAC_REG_CLEAR_CODE                0x03
#define DAC_REG_USER_GAIN                 0x04
#define DAC_REG_USER_OFFSET               0x05
#define DAC_REG_DAC_CONFIG                0x06
#define DAC_REG_SW_LDAC                   0x07
#define DAC_REG_KEY                       0x08
#define DAC_REG_GP_CONFIG1                0x09
#define DAC_REG_GP_CONFIG2                0x0A
#define DAC_REG_DCDC_CONFIG1              0x0B
#define DAC_REG_DCDC_CONFIG2              0x0C
#define DAC_REG_GPIO_CONFIG               0x0D
#define DAC_REG_GPIO_DATA                 0x0E
#define DAC_REG_WDT_CONFIG                0x0F
#define DAC_REG_DIGITAL_DIAG_CONFIG       0x10
#define DAC_REG_ADC_CONFIG                0x11
#define DAC_REG_FAULT_PIN_CONFIG          0x12
#define DAC_REG_TWO_STAGE_READBACK_SELECT 0x13
#define DAC_REG_DIGITAL_DIAG_RESULTS      0x14
#define DAC_REG_ANALOG_DIAG_RESULTS       0x15
#define DAC_REG_STATUS                    0x16
#define DAC_REG_CHIP_ID                   0x17
#define DAC_REG_FREQ_MONITOR              0x18
#define DAC_REG_DEVICE_ID_0               0x19
#define DAC_REG_DEVICE_ID_1               0x1A
#define DAC_REG_DEVICE_ID_2               0x1B
#define DAC_REG_DEVICE_ID_3               0x1C

/* AD5753_REG_DAC_CONFIG */
#define DAC_CONFIG_RANGE_MSK             GENMASK(3, 0)
#define DAC_CONFIG_RANGE_MODE(x)         (((x) & 0xF) << 0)
#define DAC_CONFIG_OVRNG_EN_MSK          BIT(4)
#define DAC_CONFIG_OVRNG_EN_MODE(x)      (((x) & 0x1) << 4)
#define DAC_CONFIG_INT_EN_MSK            BIT(5)
#define DAC_CONFIG_INT_EN_MODE(x)        (((x) & 0x1) << 5)
#define DAC_CONFIG_OUT_EN_MSK            BIT(6)
#define DAC_CONFIG_OUT_EN_MODE(x)        (((x) & 0x1) << 6)
#define DAC_CONFIG_RSET_EXT_EN_MSK       BIT(7)
#define DAC_CONFIG_RSET_EXT_EN_MODE(x)   (((x) & 0x1) << 7)
#define DAC_CONFIG_SR_EN_MSK             BIT(8)
#define DAC_CONFIG_SR_EN_MODE(x)         (((x) & 0x1) << 8)
#define DAC_CONFIG_SR_CLOCK_MSK          GENMASK(12, 9)
#define DAC_CONFIG_SR_CLOCK_MODE(x)      (((x) & 0xF) << 9)
#define DAC_CONFIG_SR_STEP_MSK           GENMASK(15, 13)
#define DAC_CONFIG_SR_STEP_MODE(x)       (((x) & 0x7) << 13)

/* AD5753_REG_DAC_CONFIG available ranges */
#define DAC_CONFIG_RANGE_5V_UNI          0x0
#define DAC_CONFIG_RANGE_10V_UNI         0x1
#define DAC_CONFIG_RANGE_5V_BIP          0x2
#define DAC_CONFIG_RANGE_10V_BIP         0x3
#define DAC_CONFIG_RANGE_20mA_UNI        0x8
#define DAC_CONFIG_RANGE_24mA_UNI        0x9
#define DAC_CONFIG_RANGE_4_24mA_UNI      0xA
#define DAC_CONFIG_RANGE_20mA_BIP        0xB
#define DAC_CONFIG_RANGE_24mA_BIP        0xC
#define DAC_CONFIG_RANGE_22mA_OVR        0xD

// Checks if a range is bipolar.
#define DAC_CONFIG_RANGE_IS_BIPOLAR(x)   (               \
										 ((x) == 0x2) || \
										 ((x) == 0x3) || \
										 ((x) == 0xB) || \
										 ((x) == 0xC)    \
										 )
// True if it is a current range
#define DAC_CONFIG_CURRENT_MODE(x)       (((x) & 0x8) > 0)


/* AD5753_REG_DIGITAL_DIAG_RESULTS */
#define DAC_DI_DIAG_SPI_CRC_ERR         BIT(0)
#define DAC_DI_DIAG_SLIPBIT_ERR         BIT(1)
#define DAC_DI_DIAG_SCLK_COUNT_ERR      BIT(2)
#define DAC_DI_DIAG_INVALID_SPI_ACC_ERR BIT(4)
#define DAC_DI_DIAG_CAL_MEM_CRC_ERR     BIT(5)
#define DAC_DI_DIAG_INV_DAC_CHECK_ERR   BIT(6)
#define DAC_DI_DIAG_DAC_LATCH_MON_ERR   BIT(8)
#define DAC_DI_DIAG_3WI_RC_ERR          BIT(9)
#define DAC_DI_DIAG_WDT_ERR             BIT(11)
#define DAC_DI_DIAG_ERR_3WI             BIT(12)
#define DAC_DI_DIAG_RES_OCCURRED        BIT(13)
#define DAC_DI_DIAG_SLEW_BUSY           BIT(14)
#define DAC_DI_DIAG_CAL_MEM_UNREFRESHED BIT(15)

/* AD5753_REG_ANALOG_DIAG_RESULTS */
#define DAC_AN_DIAG_VLDO_ERR            BIT(0)
#define DAC_AN_DIAG_INT_AVCC_ERR        BIT(1)
#define DAC_AN_DIAG_REFIN_ERR           BIT(2)
#define DAC_AN_DIAG_REFOUT_ERR          BIT(3)
#define DAC_AN_DIAG_MAIN_DIE_TEMP_ERR   BIT(4)
#define DAC_AN_DIAG_DCDC_DIE_TEMP_ERR   BIT(5)
#define DAC_AN_DIAG_VOUT_SC_ERR         BIT(6)
#define DAC_AN_DIAG_IOUT_OC_ERR         BIT(7)
#define DAC_AN_DIAG_DCDC_P_PWR_ERR      BIT(9)
#define DAC_AN_DIAG_DCDC_P_SC_ERR       BIT(12)
#define DAC_AN_DIAG_VIOUT_OV_ERR        BIT(13)

/* AD5753_REG_ADC_CONFIG */
#define DAC_CONFIG_ADC_IP_SELECT_MSK             GENMASK(4, 0)
#define DAC_CONFIG_ADC_IP_SELECT_MODE(x)         (((x) & 0x1F) << 0)
#define DAC_CONFIG_SEQUENCE_DATA_MSK             GENMASK(7, 5)
#define DAC_CONFIG_SEQUENCE_DATA_MODE(x)         (((x) & 0x7) << 5)
#define DAC_CONFIG_SEQUENCE_COMMAND_MSK          GENMASK(10, 8)
#define DAC_CONFIG_SEQUENCE_COMMAND_MODE(x)      (((x) & 0x7) << 8)
#define DAC_CONFIG_PPC_BUF_MSK                   BIT(11)
#define DAC_CONFIG_PPC_BUF_EN(x)                 (((x) & 0x1) << 11)

/* AD5753_REG_GP_CONFIG1 */
#define DAC_GP_CONFIG1_OSC_STOP_DETECT_EN_MSK        BIT(2)
#define DAC_GP_CONFIG1_OSC_STOP_DETECT_EN_MODE(x)    (((x) & 0x1) << 2)
#define DAC_GP_CONFIG1_SPI_DIAG_QUIET_EN_MSK         BIT(3)
#define DAC_GP_CONFIG1_SPI_DIAG_QUIET_EN_MODE(x)     (((x) & 0x1) << 3)
#define DAC_GP_CONFIG1_CLEAR_NOW_EN_MSK              BIT(4)
#define DAC_GP_CONFIG1_CLEAR_NOW_EN_MODE(x)          (((x) & 0x1) << 4)
#define DAC_GP_CONFIG1_NEG_OFFSET_EN_MSK             BIT(5)
#define DAC_GP_CONFIG1_NEG_OFFSET_EN_MODE(x)         (((x) & 0x1) << 5)
#define DAC_GP_CONFIG1_HART_EN_MSK                   BIT(6)
#define DAC_GP_CONFIG1_HART_EN_MODE(x)               (((x) & 0x1) << 6)
#define DAC_GP_CONFIG1_CLKOUT_FREQ_MSK               GENMASK(9, 7)
#define DAC_GP_CONFIG1_CLKOUT_FREQ_MODE(x)           (((x) & 0x7) << 7)
#define DAC_GP_CONFIG1_CLKOUT_CONFIG_MSK             GENMASK(11, 10)
#define DAC_GP_CONFIG1_CLKOUT_CONFIG_MODE(x)         (((x) & 0x3) << 10)
#define DAC_GP_CONFIG1_SET_TEMP_THRESHOLD_MSK        GENMASK(13, 12)
#define DAC_GP_CONFIG1_SET_TEMP_THRESHOLD_MODE(x)    (((x) & 0x3) << 12)

/* AD5753_REG_GPIO_CONFIG */
#define DAC_REG_GPIO_CONFIG_GPIO_2_CFG_MSK           GENMASK(5, 4)
#define DAC_REG_GPIO_CONFIG_GPIO_2_CFG_MODE(x)       (((x) & 0x3) << 4)
#define DAC_REG_GPIO_CONFIG_GPIO_1_CFG_MSK           GENMASK(3, 2)
#define DAC_REG_GPIO_CONFIG_GPIO_1_CFG_MODE(x)       (((x) & 0x3) << 2)
#define DAC_REG_GPIO_CONFIG_GPIO_0_CFG_MSK           GENMASK(1, 0)
#define DAC_REG_GPIO_CONFIG_GPIO_0_CFG_MODE(x)       (((x) & 0x3) << 0)

/* AD5753_REG_GPIO_DATA */
#define DAC_REG_GPIO_DATA_GPI_2_READ_MSK             BIT(5)
#define DAC_REG_GPIO_DATA_GPI_2_READ_MODE(x)         (((x) & 0x1) << 5)
#define DAC_REG_GPIO_DATA_GPO_2_WRITE_MSK            BIT(4)
#define DAC_REG_GPIO_DATA_GPO_2_WRITE_MODE(x)         (((x) & 0x1) << 4)
#define DAC_REG_GPIO_DATA_GPI_1_READ_MSK             BIT(3)
#define DAC_REG_GPIO_DATA_GPI_1_READ_MODE(x)         (((x) & 0x1) << 3)
#define DAC_REG_GPIO_DATA_GPO_1_WRITE_MSK            BIT(2)
#define DAC_REG_GPIO_DATA_GPO_1_WRITE_MODE(x)         (((x) & 0x1) << 2)
#define DAC_REG_GPIO_DATA_GPI_0_READ_MSK             BIT(1)
#define DAC_REG_GPIO_DATA_GPI_0_READ_MODE(x)         (((x) & 0x1) << 1)
#define DAC_REG_GPIO_DATA_GPO_0_WRITE_MSK            BIT(0)
#define DAC_REG_GPIO_DATA_GPO_0_WRITE_MODE(x)         (((x) & 0x1) << 0)


/* AD5753_REG_DCDC_CONFIG1 */
#define DAC_DCDC_CONFIG1_DCDC_VPROG_MSK              GENMASK(4, 0)
#define DAC_DCDC_CONFIG1_DCDC_VPROG_MODE(x)          (((x) & 0x1F) << 0)
#define DAC_DCDC_CONFIG1_DCDC_MODE_MSK               GENMASK(6, 5)
#define DAC_DCDC_CONFIG1_DCDC_MODE_MODE(x)           (((x) & 0x3) << 5)

/* AD5753_REG_DCDC_CONFIG2 */
#define DAC_DCDC_CONFIG2_NEG_EN_MSK                  BIT(0)
#define DAC_DCDC_CONFIG2_NEG_EN_MODE(x)              (((x) & 0x1) << 0)
#define DAC_DCDC_CONFIG2_ILIMIT_MSK                  GENMASK(3, 1)
#define DAC_DCDC_CONFIG2_ILIMIT_MODE(x)              (((x) & 0x7) << 1)
#define DAC_DCDC_CONFIG2_ADC_CONTROL_DIAG_MSK        GENMASK(5, 4)
#define DAC_DCDC_CONFIG2_ADC_CONTROL_DIAG_MODE(x)    (((x) & 0x3) << 4)
#define DAC_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MSK       BIT(6)
#define DAC_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MODE(x)   (((x) & 0x1) << 6)
#define DAC_DCDC_CONFIG2_SHORT_DEGLITCH_MSK          BIT(7)
#define DAC_DCDC_CONFIG2_SHORT_DEGLITCH_MODE(x)      (((x) & 0x1) << 7)
#define DAC_DCDC_CONFIG2_READ_COMP_DIS_MSK           BIT(10)
#define DAC_DCDC_CONFIG2_READ_COMP_DIS_MODE(x)       (((x) & 0x1) << 10)
#define DAC_DCDC_CONFIG2_INTR_SAT_3WI_MSK            BIT(11)
#define DAC_DCDC_CONFIG2_BUSY_3WI_MSK                BIT(12)

/* AD5753_REG_STATUS */
#define DAC_STATUS_ADC_DATA_MSK              GENMASK(11, 0)
#define DAC_STATUS_ADC_CH_MSK                GENMASK(16, 12)
#define DAC_STATUS_ADC_BUSY_MSK              BIT(17)
#define DAC_STATUS_WDT_STATUS_MSK            BIT(18)
#define DAC_STATUS_ANA_DIAG_STATUS_MSK       BIT(19)
#define DAC_STATUS_DIG_DIAG_STATUS_MSK       BIT(20)

#define DAC_REG_WRITE(x) ((0x80) | (x&0x1F))
#define DAC_SPI_CLOCK_DIVIDER   BCM2835_SPI_CLOCK_DIVIDER_512

#define DAC_CRC8_POLY    0x07

uint8_t dac_compute_crc(uint8_t *data, uint8_t data_size);
spi_device get_dac_dev(uint8_t slot, uint8_t ch);
/* AD5753 Low-level functions                 */
exit_code transfer_dac_spi(uint8_t* buf, uint8_t* retbuf);
exit_code dac_reg_write(spi_device dev, uint8_t reg_addr, uint16_t data);
exit_code dac_reg_read(spi_device dev, uint8_t reg_addr, uint16_t *read_data);
exit_code dac_reg_read32(spi_device dev, uint8_t reg_addr, uint32_t *read_data);
exit_code dac_reg_write_mask(spi_device dev, uint8_t reg_addr, uint32_t mask, uint16_t data);

exit_code dac_adc_single_conversion(spi_device dev, uint8_t ip_select, uint16_t* adc_data);

exit_code dac_startup_sequence(spi_device dev);

exit_code dac_set_new_range(spi_device dev, uint8_t range);
exit_code dac_set_dac_code(spi_device dev, uint8_t range, uint16_t code);

exit_code dac_pause_output(spi_device dev, uint16_t* current_dac_code);
#endif
