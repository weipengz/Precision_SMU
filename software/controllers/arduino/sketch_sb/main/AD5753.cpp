/*
AD5753.h - AD5753 DAC functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "AD5753.h"

SPISettings ad5753_spi_settings (AD5753_SPI_CLOCK, MSBFIRST, SPI_MODE1);

exit_code transfer_ad5753_spi(uint8_t* buf, uint8_t* retbuf)
{
    exit_code ret = SUCCESS;

    buf[3] = ad5753_compute_crc8(buf, 3);

    SPI.beginTransaction(ad5753_spi_settings);
    for(int i; i<4; i++){
        retbuf[i] = SPI.transfer(buf[i]);
    }
    SPI.endTransaction();
    bool crc_check = (
        (retbuf[3] ^
        ad5753_compute_crc8(retbuf, 3))
        == 0x00);

    if(!crc_check) ret |= SPI_FAILURE;

    return ret;
}

exit_code ad5753_reg_write(spi_device dev, uint8_t reg_address, uint16_t data)
{

    exit_code ret = SUCCESS;

    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(reg_address);
    buf[1] = (data >> 8);
    buf[2] = (data & 0xFF);

    ss_select(dev);
    uint8_t retbuf[4];
    transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);

    return ret;
}

exit_code ad5753_reg_read(spi_device dev, uint8_t reg_address, uint16_t *read_data)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    //    ad5753_reg_write(AD5753_REG_TWO_STAGE_READBACK_SELECT, reg_address & 0x1F);
    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(AD5753_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x0;
    buf[2] = (reg_address & 0x1F);

    uint8_t retbuf[4];
    ss_select(dev);

    // Ignore faults in this write operation (SDO should be invalid)
    transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);

    // Send NOP
    buf[0] = AD5753_REG_WRITE(AD5753_REG_NOP);
    buf[1] = 0x00;
    buf[2] = 0x00;

    ss_fast_select(dev);
    // ret |= transfer_ad5753_spi(buf, retbuf);
    transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);

    *read_data = (retbuf[1] << 8) | retbuf[2];

    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1));  // check slip bit
    bool fault = ((retbuf[0] >> 5) & 0x1);  // get fault pin status

    ret |= spi_fault * SPI_FAILURE;
    ret |= fault * AD5753_FAULT;

    return ret;
}

exit_code ad5753_reg_read32(spi_device dev, uint8_t reg_address, uint32_t *read_data)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    //    ad5753_reg_write(AD5753_REG_TWO_STAGE_READBACK_SELECT, reg_address & 0x1F);
    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(AD5753_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x0;
    buf[2] = (reg_address & 0x1F);

    uint8_t retbuf[4];
    ss_select(dev);
    transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);

    // Send NOP
    buf[0] = AD5753_REG_WRITE(AD5753_REG_NOP);
    buf[1] = 0x00;
    buf[2] = 0x00;

    ss_fast_select(dev);
    ret |= transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);

    *read_data = retbuf[0];
    *read_data <<= 8;
    *read_data |= retbuf[1];
    *read_data <<= 8;
    *read_data |= retbuf[2];
    // *read_data = (retbuf[0] << 16) | (retbuf[1] << 8) | retbuf[2];

    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1));  // check slip bit
    bool fault = ((retbuf[0] >> 5) & 0x1);  // get fault pin status

    ret |= spi_fault * SPI_FAILURE;
    ret |= fault * AD5753_FAULT;

    return ret;
}

exit_code ad5753_reg_write_mask(spi_device dev,
                                uint8_t reg_address,
                                uint32_t mask,
                                uint16_t data)
{
    uint16_t reg_data;
    exit_code ret = SUCCESS;

    ret |= ad5753_reg_read(dev, reg_address, &reg_data);
    reg_data &= ~mask;
    reg_data |= data;
    ret |= ad5753_reg_write(dev, reg_address, reg_data);

    return ret;
}

exit_code ad5753_startup_sequence(spi_device dev)
/* Following Fig. 92 on page 53 the data sheet */
{
    exit_code ret = SUCCESS;
    // Software Reset
    ret |= ad5753_reg_write(dev, AD5753_REG_KEY, 0x15FA);  // first of two keys to initiate a software reset.
    EARLY_RETURN(ret)
    ret |= ad5753_reg_write(dev, AD5753_REG_KEY, 0xAF51);  // second of two keys to initiate a software reset.
    EARLY_RETURN(ret)
    delayMicroseconds(500);

    // Refresh Calibration Memory
    ret |= ad5753_reg_write(dev, AD5753_REG_KEY, 0xFCBA);  // CAL MEM REFRESH
    EARLY_RETURN(ret)
    delayMicroseconds(1250);

    uint16_t diag_results;
    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
        // Serial.println("ret: "+String(ret,DEC)+", diag_results: "+String(diag_results,DEC));
    } while (!ret && (diag_results & AD5753_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    EARLY_RETURN(ret)

    // Clear RESET_OCCURED bit
    ret |= ad5753_reg_write(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, BIT(13));
    EARLY_RETURN(ret)

    // Configure CLKOUT
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_GP_CONFIG1,
                          AD5753_GP_CONFIG1_CLKOUT_CONFIG_MSK,
                          AD5753_GP_CONFIG1_CLKOUT_CONFIG_MODE(0x1)); // Enable clock
    EARLY_RETURN(ret)

    // Setup DC-to-DC Converter
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DCDC_CONFIG2,
                          (AD5753_DCDC_CONFIG2_NEG_EN_MSK|
                           AD5753_DCDC_CONFIG2_ILIMIT_MSK|
                           AD5753_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MSK),
                          (AD5753_DCDC_CONFIG2_NEG_EN_MODE(0x1)|  // Enables negative DC-DC
                           AD5753_DCDC_CONFIG2_ILIMIT_MODE(0x0)|  // 150 mA maximum current from DCDC (default).
                           AD5753_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MODE(0X0)) // Disables pulldown resistor at VIout
                         );
    EARLY_RETURN(ret)

    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    uint16_t dcdc_status;
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & AD5753_DCDC_CONFIG2_BUSY_3WI_MSK));
    EARLY_RETURN(ret)

    // Enable DC-to-DC Converter
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DCDC_CONFIG1,
                          AD5753_DCDC_CONFIG1_DCDC_MODE_MSK,
                          AD5753_DCDC_CONFIG1_DCDC_MODE_MODE(0x2)); // 0x1 for Current mode; 0x2 for Voltage mode
    EARLY_RETURN(ret)

    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & AD5753_DCDC_CONFIG2_BUSY_3WI_MSK));
    EARLY_RETURN(ret)

    // Write DAC code 0
    ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x0);
    EARLY_RETURN(ret)

    // No need for software LDAC because LDAC_N is pinned low.
    // TODO: revisit

    // Set DAC Mode
    ad5753_reg_write_mask(dev,
                          AD5753_REG_DAC_CONFIG,
                          (AD5753_DAC_CONFIG_RANGE_MSK |
                           AD5753_DAC_CONFIG_INT_EN_MSK |
                           AD5753_DAC_CONFIG_OUT_EN_MSK),
                          (AD5753_DAC_CONFIG_RANGE_MODE(0x0)  |  // 0 to 20mA
                           AD5753_DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           AD5753_DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output

    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & AD5753_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    EARLY_RETURN(ret)
    delayMicroseconds(200);

    // Write DAC code
    ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x0);  // zero-scale code
    EARLY_RETURN(ret)

    // Enable output to calibration (reference) resistor via the ADG5236 switch
    // Write GPIO0, GPIO1 and GPIO2 to 1
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_GPIO_DATA,
                          (AD5753_REG_GPIO_DATA_GPO_0_WRITE_MSK |
                           AD5753_REG_GPIO_DATA_GPO_1_WRITE_MSK |
                           AD5753_REG_GPIO_DATA_GPO_2_WRITE_MSK),
                          (AD5753_REG_GPIO_DATA_GPO_0_WRITE_MODE(0x1) |
                           AD5753_REG_GPIO_DATA_GPO_1_WRITE_MODE(0x1) |
                           AD5753_REG_GPIO_DATA_GPO_2_WRITE_MODE(0x1))
                         );
    EARLY_RETURN(ret)

    // Configure GPIO0, GPIO1, and GPIO2 as outputs
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_GPIO_CONFIG,
                          (AD5753_REG_GPIO_CONFIG_GPIO_0_CFG_MSK |
                           AD5753_REG_GPIO_CONFIG_GPIO_1_CFG_MSK |
                           AD5753_REG_GPIO_CONFIG_GPIO_2_CFG_MSK),
                          (AD5753_REG_GPIO_CONFIG_GPIO_0_CFG_MODE(0x1) |
                           AD5753_REG_GPIO_CONFIG_GPIO_1_CFG_MODE(0x1) |
                           AD5753_REG_GPIO_CONFIG_GPIO_2_CFG_MODE(0x1))
                         );

    EARLY_RETURN(ret)

    delayMicroseconds(1250);

    #if SERIAL_DEBUG and REG_DEBUG
        ad5753_reg_read(dev, AD5753_REG_STATUS, &diag_results);
        ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
        ad5753_reg_read(dev, AD5753_REG_ANALOG_DIAG_RESULTS, &diag_results);
    #endif

    return ret;
}

exit_code ad5753_set_new_range(spi_device dev, uint8_t range)
{
    exit_code ret = SUCCESS;
    bool bipolar_range;
    uint16_t diag_results, dcdc_status;

    uint16_t dac_input_reg;
    ret |= ad5753_reg_read(dev, AD5753_REG_DAC_CONFIG, &dac_input_reg);
    bipolar_range = AD5753_DAC_CONFIG_RANGE_IS_BIPOLAR(
                        AD5753_DAC_CONFIG_RANGE_MODE(dac_input_reg));
    EARLY_RETURN(ret)

    // Write 0V/0mA code.
    if (bipolar_range)
    {
        ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x8000);
    } else
    {
        ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x0000);
    }
    EARLY_RETURN(ret)

    /* Write to the DAC_CONFIG register. Disable the output (OUT_EN = 0)
     * and set the new output range. Keep the INT_EN bit set. Wait 500 μs
     * minimum before proceeding to Step 3 to allow time for internal
     * calibrations to complete.
     */

    // Set DAC Mode
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DAC_CONFIG,
                          (AD5753_DAC_CONFIG_RANGE_MSK |
                           AD5753_DAC_CONFIG_INT_EN_MSK |
                           AD5753_DAC_CONFIG_OUT_EN_MSK),
                          (AD5753_DAC_CONFIG_RANGE_MODE(range) |  // Select Range
                           AD5753_DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           AD5753_DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output
    EARLY_RETURN(ret)
    delayMicroseconds(200);

    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & AD5753_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    EARLY_RETURN(ret)

    uint8_t dc_dc_mode;

    if (AD5753_DAC_CONFIG_CURRENT_MODE(range))
    {
        dc_dc_mode = 0x1;
    }
    else
    {
        dc_dc_mode = 0x2;
    }

    // Change DC-to-DC Converter setting
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DCDC_CONFIG1,
                          AD5753_DCDC_CONFIG1_DCDC_MODE_MSK,
                          AD5753_DCDC_CONFIG1_DCDC_MODE_MODE(dc_dc_mode)); // 0x1 for Current mode; 0x2 for Voltage mode
    EARLY_RETURN(ret)

    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & AD5753_DCDC_CONFIG2_BUSY_3WI_MSK));
    EARLY_RETURN(ret)

    /* Write Code 0x0000 or the case of bipolar ranges,
     * write Code 0x8000 to the DAC_INPUT register.
     * It is important that this step be completed even if the contents
     * of the DAC_ INPUT register do not change.
     */

    bipolar_range = AD5753_DAC_CONFIG_RANGE_IS_BIPOLAR(range);

    // Write 0V/0mA code.
    if (bipolar_range)
    {
        ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x8000);
    } else
    {
        ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x0000);
    }

    // One last fault check
    ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    return ret;
}

exit_code ad5753_pause_output(spi_device dev, uint16_t* current_dac_code)
{
    exit_code ret = SUCCESS;
    bool bipolar_range;
    uint16_t diag_results;

    uint16_t dac_input_reg;
    ret |= ad5753_reg_read(dev, AD5753_REG_DAC_CONFIG, &dac_input_reg);
    bipolar_range = AD5753_DAC_CONFIG_RANGE_IS_BIPOLAR(
                        AD5753_DAC_CONFIG_RANGE_MODE(dac_input_reg));
    EARLY_RETURN(ret)

    ret |= ad5753_reg_read(dev, AD5753_REG_DAC_INPUT, current_dac_code);
    EARLY_RETURN(ret)

    // Write 0V/0mA code.
    if (bipolar_range)
    {
        ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x8000);
    } else
    {
        ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, 0x0000);
    }
    EARLY_RETURN(ret)

    /* Write to the DAC_CONFIG register. Disable the output (OUT_EN = 0)
     * and set the new output range. Keep the INT_EN bit set. Wait 500 μs
     * minimum before proceeding to Step 3 to allow time for internal
     * calibrations to complete.
     */

    // Set DAC Mode
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DAC_CONFIG,
                          (AD5753_DAC_CONFIG_INT_EN_MSK |
                           AD5753_DAC_CONFIG_OUT_EN_MSK),
                          (AD5753_DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           AD5753_DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output
    EARLY_RETURN(ret)
    delayMicroseconds(200);

    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & AD5753_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    EARLY_RETURN(ret)

    return ret;
}

exit_code ad5753_set_dac_code(spi_device dev, uint8_t range, uint16_t dac_code)
{
    // Following instructions from page 52 of the AD5753 manual (rev. 0)

    exit_code ret = SUCCESS;
    uint8_t previous_range;
    bool bipolar_range = false;
    uint16_t dac_input_reg;
    uint16_t diag_results;

    /* Write to the DAC_INPUT register. Set the output to 0mA or 0V. */
    // Verify zero-scale.

    ret |= ad5753_reg_read(dev, AD5753_REG_DAC_CONFIG, &dac_input_reg);
    previous_range = AD5753_DAC_CONFIG_RANGE_MODE(dac_input_reg);
    EARLY_RETURN(ret)

    if (range != previous_range) ret |= ad5753_set_new_range (dev, range);
    EARLY_RETURN(ret)

    /* Reload the DAC_CONFIG register word from Step 2 and
     * set the OUT_EN bit to 1 to enable the output.
     */
    ret |= ad5753_reg_write_mask(dev,
                          AD5753_REG_DAC_CONFIG,
                          (AD5753_DAC_CONFIG_OUT_EN_MSK),
                          (AD5753_DAC_CONFIG_OUT_EN_MODE(0x1)));  // Enable Output
    EARLY_RETURN(ret)


    /* Write the required DAC code to the DAC_INPUT register.
     */
    ret |= ad5753_reg_write(dev, AD5753_REG_DAC_INPUT, dac_code);
    EARLY_RETURN(ret)

    // One last fault check
    ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    EARLY_RETURN(ret)

    return ret;
}
