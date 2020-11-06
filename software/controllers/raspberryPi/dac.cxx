/*
dac.cxx - AD5753 DAC functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "dac.h"

#if STD_DEBUG and DAC_DEBUG
    #define SIMPLE_DEBUG(x) printf((x)); printf("\n");
#else
    #define SIMPLE_DEBUG(x);
#endif

uint8_t dac_compute_crc(uint8_t *data, uint8_t data_size)
{
    uint8_t crc = 0;
    while (data_size>0) {
        for (uint8_t i = 0x80; i != 0; i >>= 1) {
            if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
                crc <<= 1;
                crc ^= DAC_CRC8_POLY;
            } 
            else 
            {
                crc <<= 1;
            }
        }
        data++;
        data_size--;
    }
    return crc;
}

spi_device get_dac_dev(uint8_t slot, uint8_t ch)
{
    #define CS_NUM 10
    uint8_t cs_pin_list[CS_NUM] = {
        MONCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0,
        DADCS
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        LOW,
        BIT2_FILT(ch),
        BIT1_FILT(ch),
        BIT0_FILT(ch),
        BIT3_FILT(slot),
        BIT2_FILT(slot),
        BIT1_FILT(slot),
        BIT0_FILT(slot),
        HIGH
    };
    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;
    spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;
    #undef CS_NUM
    return ret;
}

exit_code transfer_dac_spi(uint8_t* buf, uint8_t* retbuf)
{
    exit_code ret = SUCCESS;
    buf[3] = dac_compute_crc(buf, 3);
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI-DAC-Sending:");
        for(int i = 0; i < 4; i++){
            printf("%X-",buf[i]);
        }
        printf("\n");
    #endif
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
    bcm2835_spi_setClockDivider(DAC_SPI_CLOCK_DIVIDER);
    delayMicroseconds(10);
    
    for (uint8_t i = 0; i < 4; i++)
    {
        retbuf[i] = bcm2835_spi_transfer(buf[i]);
    }
    bcm2835_spi_end();

    bool crc_check = ((retbuf[3]^dac_compute_crc(retbuf,3)) == 0x00);
    if(!crc_check) ret |= SPI_FAILURE;
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI-DAC-Received: ");
        for(int i = 0; i < 4; i++){
            printf("%02X-",retbuf[i]);
        }
        printf("(%02X)/",dac_compute_crc(retbuf, 3));
        if (crc_check) {
            printf("OK\n");
        } else {
            printf("BAD\n");
        }
    #endif
    return ret;
}

exit_code dac_reg_write(spi_device dev, uint8_t reg_addr, uint16_t data)
{
    exit_code ret = SUCCESS;
    uint8_t buf[4];
    buf[0] = DAC_REG_WRITE(reg_addr);
    buf[1] = uint8_t(data >> 8);
    buf[2] = uint8_t(data & 0xFF);
    ss_select(dev);
    
    uint8_t retbuf[4];
    transfer_dac_spi(buf, retbuf);
    
    ss_deselect(dev);
    delayMicroseconds(10);
    
    #if STD_DEBUG and REG_DEBUG
        printf("Writing register 0x%02X: %02X\n",reg_addr,data);
    #endif
    return ret;
}

exit_code dac_reg_read(spi_device dev, uint8_t reg_addr, uint16_t *read_data)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    uint8_t buf[4];
    buf[0] = DAC_REG_WRITE(DAC_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x0;
    buf[2] = (reg_addr&0x1F);
    buf[3] = 0x00;
    
    uint8_t retbuf[4];
    ss_select(dev);
    // Ignore faults in this write operation (SDO should be invalid)
    transfer_dac_spi(buf, retbuf);
    ss_deselect(dev);
    delayMicroseconds(5);
    
    // Send NOP
    buf[0] = DAC_REG_WRITE(DAC_REG_NOP);
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    ss_fast_select(dev);
    transfer_dac_spi(buf, retbuf);
    ss_deselect(dev);
    delayMicroseconds(5);
    
    *read_data = (retbuf[1] << 8) | retbuf[2];
    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1));  // check slip bit
    bool fault = ((retbuf[0] >> 5) & 0x1);  // get fault pin status
    #if STD_DEBUG and REG_DEBUG
        printf("Reading register 0x%02X: (DATA,FAULT,SPIFAULT) %04X/%d/%d\n",reg_addr,*read_data,fault,spi_fault);
    #endif
    ret |= spi_fault*SPI_FAILURE;
    ret |= fault*DAC_FAULT;
    #if STD_DEBUG and DAC_DEBUG
        uint32_t _status_data;
        uint16_t _diag_data;
        if (fault > 0 &&
            reg_addr != DAC_REG_DIGITAL_DIAG_RESULTS &&
            reg_addr != DAC_REG_STATUS &&
            reg_addr != DAC_REG_ANALOG_DIAG_RESULTS)
        {
            printf("AD5753 FAULT...\n");
            dac_reg_read32(dev, DAC_REG_STATUS, &_status_data);
            printf("STATUS: %02X\n",_status_data);
            dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &_diag_data);
            printf("DIGITAL_DIAG_RESULTS: %02X\n",_diag_data);
            dac_reg_read(dev, DAC_REG_ANALOG_DIAG_RESULTS, &_diag_data);
            printf("ANALOG_DIAG_RESULTS: %02X\n",_diag_data);
        }
    #endif
    return ret;
}

exit_code dac_reg_read32(spi_device dev, uint8_t reg_addr, uint32_t *read_data)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    uint8_t buf[4];
    buf[0] = DAC_REG_WRITE(DAC_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x00;
    buf[2] = (reg_addr & 0x1F);
    uint8_t retbuf[4];
    ss_select(dev);
    transfer_dac_spi(buf, retbuf);
    ss_deselect(dev);
    delayMicroseconds(5);
    
    // Send NOP
    buf[0] = DAC_REG_WRITE(DAC_REG_NOP);
    buf[1] = 0x00;
    buf[2] = 0x00;
    ss_fast_select(dev);
    ret |= transfer_dac_spi(buf, retbuf);
    ss_deselect(dev);
    delayMicroseconds(5);
    
    *read_data = retbuf[0];
    *read_data <<= 8;
    *read_data |= retbuf[1];
    *read_data <<= 8;
    *read_data |= retbuf[2];
    // *read_data = (retbuf[0] << 16) | (retbuf[1] << 8) | retbuf[2];
    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1));  // check slip bit
    bool fault = ((retbuf[0] >> 5) & 0x1);  // get fault pin status
    #if STD_DEBUG and REG_DEBUG
        printf("Reading register 0x%02X: (DATA,FAULT,SPIFAULT) %06X/%d/%d\n",reg_addr,*read_data,fault,spi_fault);
    #endif
    ret |= spi_fault*SPI_FAILURE;
    ret |= fault*DAC_FAULT;
    #if STD_DEBUG and DAC_DEBUG
        uint32_t _status_data;
        uint16_t _diag_data;
        if (fault > 0 &&
            reg_addr != DAC_REG_DIGITAL_DIAG_RESULTS &&
            reg_addr != DAC_REG_STATUS &&
            reg_addr != DAC_REG_ANALOG_DIAG_RESULTS)
        {
            printf("AD5753 FAULT...\n");
            dac_reg_read32(dev, DAC_REG_STATUS, &_status_data);
            printf("STATUS: %02X\n",_status_data);
            dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &_diag_data);
            printf("DIGITAL_DIAG_RESULTS: %02X\n",_diag_data);
            dac_reg_read(dev, DAC_REG_ANALOG_DIAG_RESULTS, &_diag_data);
            printf("ANALOG_DIAG_RESULTS: %02X\n",_diag_data);
        }
    #endif
    return ret;
}

exit_code dac_reg_write_mask(spi_device dev, uint8_t reg_addr, uint32_t mask, uint16_t data)
{
    uint16_t reg_data;
    exit_code ret = SUCCESS;
    ret |= dac_reg_read(dev, reg_addr, &reg_data);
    if (ret > 0) {
        #if STD_DEBUG and REG_DEBUG
            printf("Aborted write due to BAD READ: %02X, excode: 0x%02X\n",reg_data,ret);
        #endif
        return ret;
    }
    
    reg_data &= ~mask;
    reg_data |= data;
    ret |= dac_reg_write(dev, reg_addr, reg_data);
    return ret;
}

exit_code dac_adc_single_conversion(spi_device dev, uint8_t ip_select, uint16_t* adc_data)
{
    uint32_t status_reg;
    exit_code ret = SUCCESS;
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_ADC_CONFIG,
                          (DAC_CONFIG_SEQUENCE_COMMAND_MSK |
                           DAC_CONFIG_ADC_IP_SELECT_MSK),
                          (DAC_CONFIG_SEQUENCE_COMMAND_MODE(0x4)|  //Single conversion
                           DAC_CONFIG_ADC_IP_SELECT_MODE(ip_select))
                         );
    do {
        ret |= dac_reg_read32(dev, DAC_REG_STATUS, &status_reg);
    } while (!ret && (status_reg & DAC_STATUS_ADC_BUSY_MSK));
    *adc_data = status_reg & DAC_STATUS_ADC_DATA_MSK;
    #if STD_DEBUG and ADC_DEBUG
        printf("ADC(%02X):%d/%d\n",ip_select,adc_data,0xFFF);
    #endif
    return ret;
}

exit_code dac_startup_sequence(spi_device dev)
/* Following Fig. 92 on page 53 the data sheet */
{
    exit_code ret = SUCCESS;
    // Software Reset
    SIMPLE_DEBUG("AD5753 Software Reset, first key")
    ret |= dac_reg_write(dev, DAC_REG_KEY, 0x15FA);  // first of two keys to initiate a software reset.
    SIMPLE_DEBUG("AD5753 Software Reset, second key")
    ret |= dac_reg_write(dev, DAC_REG_KEY, 0xAF51);  // second of two keys to initiate a software reset.
    delayMicroseconds(500);
    // Refresh Calibration Memory
    SIMPLE_DEBUG("AD5753 Refresh Cal Memory")
    ret |= dac_reg_write(dev, DAC_REG_KEY, 0xFCBA);  // CAL MEM REFRESH
    delayMicroseconds(1250);
    uint16_t diag_results;
    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
        // Serial.println("ret: "+String(ret,DEC)+", diag_results: "+String(diag_results,DEC));
    } while (!ret && (diag_results & DAC_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    // Clear RESET_OCCURED bit
    SIMPLE_DEBUG("AD5753 Clear RESET_OCCURED bit")
    ret |= dac_reg_write(dev, DAC_REG_DIGITAL_DIAG_RESULTS, BIT(13));
    // Configure CLKOUT
    SIMPLE_DEBUG("AD5753 Configure CLKOUT")
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_GP_CONFIG1,
                          DAC_GP_CONFIG1_CLKOUT_CONFIG_MSK,
                          DAC_GP_CONFIG1_CLKOUT_CONFIG_MODE(0x1)); // Enable clock
    // Setup DC-to-DC Converter
    SIMPLE_DEBUG("AD5753 Setup DC-to-DC Converter")
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DCDC_CONFIG2,
                          (DAC_DCDC_CONFIG2_NEG_EN_MSK|
                           DAC_DCDC_CONFIG2_ILIMIT_MSK|
                           DAC_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MSK),
                          (DAC_DCDC_CONFIG2_NEG_EN_MODE(0x1)|  // Enables negative DC-DC
                           DAC_DCDC_CONFIG2_ILIMIT_MODE(0x0)|  // 150 mA maximum current from DCDC (default).
                           DAC_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MODE(0X0)) // Disables pulldown resistor at VIout
                         );
    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    uint16_t dcdc_status;
    do {
        ret |= dac_reg_read(dev, DAC_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & DAC_DCDC_CONFIG2_BUSY_3WI_MSK));
    // Enable DC-to-DC Converter
    SIMPLE_DEBUG("AD5753 Enable DC-to-DC Converter (Voltage mode)")
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DCDC_CONFIG1,
                          DAC_DCDC_CONFIG1_DCDC_MODE_MSK,
                          DAC_DCDC_CONFIG1_DCDC_MODE_MODE(0x2)); // 0x1 for Current mode; 0x2 for Voltage mode
    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & DAC_DCDC_CONFIG2_BUSY_3WI_MSK));
    // Write DAC code 0
    SIMPLE_DEBUG("AD5753 Write DAC code 0.")
    dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x0);
    // No need for software LDAC because LDAC_N is pinned low.
    // TODO: revisit
    
    // Set DAC Mode
    SIMPLE_DEBUG("AD5753 Set DAC mode to 5V Unipolar")
    dac_reg_write_mask(dev,
                          DAC_REG_DAC_CONFIG,
                          (DAC_CONFIG_RANGE_MSK |
                           DAC_CONFIG_INT_EN_MSK |
                           DAC_CONFIG_OUT_EN_MSK),
                          (DAC_CONFIG_RANGE_MODE(0x0)  |  // 0 to 20mA
                           DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output
    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & DAC_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    delayMicroseconds(200);
    // Write DAC code
    SIMPLE_DEBUG("AD5753 Write DAC code 0.")
    ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x0);  // zero-scale code
    // Enable output to calibration (reference) resistor via the ADG5236 switch
    // Write GPIO0, GPIO1 and GPIO2 to 1
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_GPIO_DATA,
                          (DAC_REG_GPIO_DATA_GPO_0_WRITE_MSK |
                           DAC_REG_GPIO_DATA_GPO_1_WRITE_MSK |
                           DAC_REG_GPIO_DATA_GPO_2_WRITE_MSK),
                          (DAC_REG_GPIO_DATA_GPO_0_WRITE_MODE(0x1) |
                           DAC_REG_GPIO_DATA_GPO_1_WRITE_MODE(0x1) |
                           DAC_REG_GPIO_DATA_GPO_2_WRITE_MODE(0x1))
                         );
    // Configure GPIO0, GPIO1, and GPIO2 as outputs
    SIMPLE_DEBUG("Configure GPIO0, GPIO1, and GPIO2 as outputs.")
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_GPIO_CONFIG,
                          (DAC_REG_GPIO_CONFIG_GPIO_0_CFG_MSK |
                           DAC_REG_GPIO_CONFIG_GPIO_1_CFG_MSK |
                           DAC_REG_GPIO_CONFIG_GPIO_2_CFG_MSK),
                          (DAC_REG_GPIO_CONFIG_GPIO_0_CFG_MODE(0x1) |
                           DAC_REG_GPIO_CONFIG_GPIO_1_CFG_MODE(0x1) |
                           DAC_REG_GPIO_CONFIG_GPIO_2_CFG_MODE(0x1))
                         );
    delayMicroseconds(1250);
    #if STD_DEBUG and REG_DEBUG
        dac_reg_read(dev, DAC_REG_STATUS, &diag_results);
        dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
        dac_reg_read(dev, DAC_REG_ANALOG_DIAG_RESULTS, &diag_results);
    #endif
    return ret;
}

exit_code dac_set_new_range(spi_device dev, uint8_t range)
{
    exit_code ret = SUCCESS;
    bool bipolar_range;
    uint16_t diag_results, dcdc_status;
    uint16_t dac_input_reg;
    ret |= dac_reg_read(dev, DAC_REG_DAC_CONFIG, &dac_input_reg);
    bipolar_range = DAC_CONFIG_RANGE_IS_BIPOLAR(
                        DAC_CONFIG_RANGE_MODE(dac_input_reg));
    // Write 0V/0mA code.
    if (bipolar_range)
    {
        ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x8000);
    } else
    {
        ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x0000);
    }
    /* Write to the DAC_CONFIG register. Disable the output (OUT_EN = 0)
     * and set the new output range. Keep the INT_EN bit set. Wait 500 μs
     * minimum before proceeding to Step 3 to allow time for internal
     * calibrations to complete.
     */
    // Set DAC Mode
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DAC_CONFIG,
                          (DAC_CONFIG_RANGE_MSK |
                           DAC_CONFIG_INT_EN_MSK |
                           DAC_CONFIG_OUT_EN_MSK),
                          (DAC_CONFIG_RANGE_MODE(range) |  // Select Range
                           DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output
    delayMicroseconds(200);
    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & DAC_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    uint8_t dc_dc_mode;
    if (DAC_CONFIG_CURRENT_MODE(range))
    {
        dc_dc_mode = 0x1;
    }
    else
    {
        dc_dc_mode = 0x2;
    }
    // Change DC-to-DC Converter setting
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DCDC_CONFIG1,
                          DAC_DCDC_CONFIG1_DCDC_MODE_MSK,
                          DAC_DCDC_CONFIG1_DCDC_MODE_MODE(dc_dc_mode)); // 0x1 for Current mode; 0x2 for Voltage mode
    /*
     * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
     * This allows the 3-wire interface communication to complete.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DCDC_CONFIG2, &dcdc_status);
    } while (!ret && (dcdc_status & DAC_DCDC_CONFIG2_BUSY_3WI_MSK));
    /* Write Code 0x0000 or the case of bipolar ranges,
     * write Code 0x8000 to the DAC_INPUT register.
     * It is important that this step be completed even if the contents
     * of the DAC_ INPUT register do not change.
     */
    bipolar_range = DAC_CONFIG_RANGE_IS_BIPOLAR(range);
    // Write 0V/0mA code.
    if (bipolar_range)
    {
        dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x8000);
    } else
    {
        dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x0000);
    }
    // One last fault check
    ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    delayMicroseconds(10);
    return ret;
}

exit_code dac_pause_output(spi_device dev, uint16_t* current_dac_code)
{
    exit_code ret = SUCCESS;
    bool bipolar_range;
    uint16_t diag_results;
    uint16_t dac_input_reg;
    ret |= dac_reg_read(dev, DAC_REG_DAC_CONFIG, &dac_input_reg);
    bipolar_range = DAC_CONFIG_RANGE_IS_BIPOLAR(
                        DAC_CONFIG_RANGE_MODE(dac_input_reg));
    ret |= dac_reg_read(dev, DAC_REG_DAC_INPUT, current_dac_code);
    // Write 0V/0mA code.
    if (bipolar_range)
    {
        ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x8000);
    } else
    {
        ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, 0x0000);
    }
    /* Write to the DAC_CONFIG register. Disable the output (OUT_EN = 0)
     * and set the new output range. Keep the INT_EN bit set. Wait 500 μs
     * minimum before proceeding to Step 3 to allow time for internal
     * calibrations to complete.
     */
    // Set DAC Mode
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DAC_CONFIG,
                          (DAC_CONFIG_INT_EN_MSK |
                           DAC_CONFIG_OUT_EN_MSK),
                          (DAC_CONFIG_INT_EN_MODE(0x1) |  // Enable Buffer
                           DAC_CONFIG_OUT_EN_MODE(0x0)));  // Disable Output
    delayMicroseconds(200);
    /*
     * Wait until the CAL_MEM_UNREFRESHED bit in the DIGITAL_DIAG_RESULTS
     * register returns to 0.
     */
    do {
        ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    } while (!ret && (diag_results & DAC_DI_DIAG_CAL_MEM_UNREFRESHED));
    // Wait until CAL_MEM_UNREFRESHED returns 0
    delayMicroseconds(200);
    return ret;
}

exit_code dac_set_dac_code(spi_device dev, uint8_t range, uint16_t dac_code)
{
    // Following instructions from page 52 of the AD5753 manual (rev. 0)
    exit_code ret = SUCCESS;
    uint8_t previous_range;
    //~ bool bipolar_range = false;
    uint16_t dac_input_reg;
    uint16_t diag_results;
    /* Write to the DAC_INPUT register. Set the output to 0mA or 0V. */
    // Verify zero-scale.
    ret |= dac_reg_read(dev, DAC_REG_DAC_CONFIG, &dac_input_reg);
    previous_range = DAC_CONFIG_RANGE_MODE(dac_input_reg);
    if (range != previous_range) ret |= dac_set_new_range (dev, range);
    /* Reload the DAC_CONFIG register word from Step 2 and
     * set the OUT_EN bit to 1 to enable the output.
     */
    ret |= dac_reg_write_mask(dev,
                          DAC_REG_DAC_CONFIG,
                          (DAC_CONFIG_OUT_EN_MSK),
                          (DAC_CONFIG_OUT_EN_MODE(0x1)));  // Enable Output
    /* Write the required DAC code to the DAC_INPUT register.
     */
    ret |= dac_reg_write(dev, DAC_REG_DAC_INPUT, dac_code);
    // One last fault check
    ret |= dac_reg_read(dev, DAC_REG_DIGITAL_DIAG_RESULTS, &diag_results);
    return ret;
}
