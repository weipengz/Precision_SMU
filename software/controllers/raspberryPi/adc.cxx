/*
adc.cxx - MAX11254 ADC functionality
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "adc.h"

spi_device get_adc_dev(uint8_t slot, uint8_t ch)
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
        LOW
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

exit_code adc_reg_write(spi_device dev, uint8_t reg_addr, uint8_t* reg_value, uint8_t reg_size)
{
    ss_select(dev);
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(ADC_SPI_CLOCK_DIVIDER);
    delayMicroseconds(10);
    
    bcm2835_spi_transfer(ADC_REG_WRITE(reg_addr));
    for (int i = 0; i < reg_size; i++)
    {
        bcm2835_spi_transfer(reg_value[i]);
    }
    bcm2835_spi_end();
    
    ss_deselect(dev);
    delayMicroseconds(10);
    
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI-MAX-Sending: %X\n",ADC_REG_WRITE(reg_addr));
        printf("SPI-MAX-Received: ");
        for(int i = 0; i<reg_size; i++){
            printf("%02X-",reg_value[i]);
        }
        printf("\n");
    #endif    
    return SUCCESS;
}

exit_code adc_reg_read(spi_device dev, uint8_t reg_addr, uint8_t* reg_value, uint8_t reg_size)
{
    ss_select(dev);
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(ADC_SPI_CLOCK_DIVIDER);
    delayMicroseconds(10);
    
    bcm2835_spi_transfer(ADC_REG_READ(reg_addr));
    for (int i = 0; i < reg_size; i++)
    {
        reg_value[i] = bcm2835_spi_transfer(0x00);
    }
    bcm2835_spi_end();
    
    ss_deselect(dev);
    delayMicroseconds(10);
    
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI-MAX-Sending: %X\n",ADC_REG_READ(reg_addr));
        printf("SPI-MAX-Received: ");
        for(int i = 0; i<reg_size; i++){
            printf("%02X-",reg_value[i]);
        }
        printf("\n");
    #endif
    return SUCCESS;
}

exit_code adc_reg_read8(spi_device dev, uint8_t reg_addr, uint8_t* reg_value)
{
    return adc_reg_read(dev, reg_addr, reg_value, 1);
}

exit_code adc_reg_read24 (spi_device dev, uint8_t reg_addr, uint32_t* reg_value)
{
    exit_code ret = SUCCESS;
    uint8_t reg_data[3];
    ret |= adc_reg_read(dev, reg_addr, reg_data, 3);
    *reg_value = ((((uint32_t) reg_data[0]) << 16) +
                  (((uint32_t) reg_data[1]) << 8) +
                  (((uint32_t) reg_data[2]) << 0));
    return ret;
}    

exit_code adc_reg_read_status(spi_device dev, uint32_t* reg_value)
{
    return adc_reg_read24(dev, ADC_REG_STAT, reg_value);
}

exit_code adc_reg_write8(spi_device dev, uint8_t reg_addr, uint8_t reg_value)
{
    uint8_t data = reg_value;
    return adc_reg_write(dev, reg_addr, &data, 1);
}

exit_code adc_reg_write_mask(spi_device dev, uint8_t reg_addr, uint8_t mask, uint8_t reg_value)
{
    uint8_t reg_data;
    exit_code ret = SUCCESS;
    ret |= adc_reg_read(dev, reg_addr, &reg_data, 1);
    reg_data &= ~mask;
    reg_data |= reg_value;
    ret |= adc_reg_write(dev, reg_addr, &reg_data, 1);
    return ret;
}

exit_code adc_command(spi_device dev, uint8_t cmd, bool fast)
{
    if(!fast){ss_select(dev);}
    else {ss_fast_select(dev);}
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(ADC_SPI_CLOCK_DIVIDER);
    delayMicroseconds(10);
    bcm2835_spi_transfer(cmd);
    bcm2835_spi_end();
    
    ss_deselect(dev);
    delayMicroseconds(10);
    return SUCCESS;
}

exit_code adc_calibrate (spi_device dev, uint8_t cal_type)
{
    exit_code ret = SUCCESS;
    cal_type &= 0b11;
    // The calibration command can only be issued in sequencer mode 1 (0b00)
    uint8_t seq_data;
    ret |= adc_reg_read8(dev, ADC_REG_SEQ, &seq_data);
    if((seq_data & ADC_SEQ_MODE_MSK) != ADC_SEQ_MODE_MODE(0b00))
    {
        //~ printf("SEQ ERROR!\n");
        ret |= CAL_FAILURE;
        return ret;
    }
    switch (cal_type) {
        case 0b00 :  // Performs a self-calibration
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL1, ADC_CTRL1_CAL_MASK, ADC_CTRL1_CAL_MODE(0b00)
                );

            adc_command(dev, ADC_CALIBRATE, true);
            delay(200);  // "A self-calibration requires 200ms to complete."
            break;
        case 0b01 :  // Performs a system-level offset calibration
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL1, ADC_CTRL1_CAL_MASK, ADC_CTRL1_CAL_MODE(0b01)
                );
            adc_command(dev, ADC_CALIBRATE, true);
            delay(100);  // "A system offset calibration requires 100ms to complete."
            break;
        case 0b10 :  // Performs a system-level full-scale calibration
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL1, ADC_CTRL1_CAL_MASK, ADC_CTRL1_CAL_MODE(0b10)
                );
            adc_command(dev, ADC_CALIBRATE, true);
            delay(100);  // "A system full-scale calibration requires 100ms to complete."
            break;
        default :
            ret |= FAILURE;
    }
    delayMicroseconds(10);
    return ret;
}

exit_code adc_swreset (spi_device dev)
{
    uint32_t stat_data;
    exit_code ret = SUCCESS;
    ret |= adc_reg_write_mask(dev, ADC_REG_CTRL1, ADC_CTRL1_PD_MASK, ADC_CTRL1_PD_MODE(0b11));// Set POWERDOWN to reset
    ret |= adc_command(dev, ADC_POWERDOWN, true);
    unsigned long initial_time = millis();
    bool reset_in_progress = true;
    do
    {
        ret |= adc_reg_read_status(dev, &stat_data);
        reset_in_progress &= (ADC_STAT_INRESET_FILTER(stat_data) ||
                              (ADC_STAT_PDSTAT_FILTER(stat_data) != 0b10));
        // This should take a maximum of 28 ms
        reset_in_progress &= (millis() - initial_time < 28);
        delay(1);
    } while (reset_in_progress);
    return ret;
}

exit_code adc_setup (spi_device dev, bool skip_reset)
{
    exit_code ret = SUCCESS;
    if (!skip_reset) ret |= adc_swreset(dev);
    ret |= adc_reg_write8(dev, ADC_REG_CTRL1,     ADC_CTRL1_DEFAULT);
    ret |= adc_reg_write8(dev, ADC_REG_CTRL2,     ADC_CTRL2_DEFAULT);
    ret |= adc_reg_write8(dev, ADC_REG_CTRL3,     ADC_CTRL3_DEFAULT);
    ret |= adc_reg_write8(dev, ADC_REG_GPIO_CTRL, ADC_GPIO_CTRL_DEFAULT);
    ret |= adc_reg_write8(dev, ADC_REG_SEQ,       ADC_SEQ_DEFAULT);
    //~ uint8_t data;
	//~ ret |= adc_reg_read8(dev,ADC_REG_SEQ,&data);
	//~ printf("%X\n",data);
    delayMicroseconds(10);
    return ret;
}

exit_code adc_prepare_convert (spi_device dev, uint8_t pga)
{
    exit_code ret = SUCCESS;
    // Set sequencer mode 1
    ret |= adc_reg_write_mask(dev, ADC_REG_SEQ, ADC_SEQ_MODE_MSK, ADC_SEQ_MODE_MODE(0b00));
    // Set pga
    switch (pga)
    {
        case 1:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b000) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 2:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b001) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 4:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b010) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 8:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b011) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 16:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b100) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 32:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b101) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 64:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b110) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        case 128:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b111) | ADC_CTRL2_PGAEN_MODE(1));
            break;
        default:
            ret |= adc_reg_write_mask(dev,
                ADC_REG_CTRL2,
                ADC_CTRL2_PGA_MASK | ADC_CTRL2_PGAEN_MASK,
                ADC_CTRL2_PGA_MODE(0b000) | ADC_CTRL2_PGAEN_MODE(0));
    }
    delayMicroseconds(10);
    return ret;
}

exit_code adc_convert (spi_device dev, uint8_t channel, uint8_t rate, uint32_t* data)
{
    exit_code ret = SUCCESS;
    channel &= 0x7;
    // Select channel
    ret |= adc_reg_write_mask(dev, ADC_REG_SEQ, ADC_SEQ_MUX_MSK, ADC_SEQ_MUX_MODE(channel));
    // Trigger conversion
    ret |= adc_command(dev, ADC_CONVERT(rate), true);
    uint32_t stat_data;
    // wait until STAT:RDY == 1, which indicates data is ready
    unsigned long initial_time = millis();
    unsigned long time_step_ms = 0;
    do
    {
        ret |= adc_reg_read24(dev, ADC_REG_STAT, &stat_data);
        //~ printf("%d\n",millis());
        // stop early if 100ms have elapsed
        if ( (millis()-initial_time) > ADC_CONVERSION_TIMEOUT_MS)
        {
            printf("Time out!\n");
            return (ret | TIMEOUT);
        }
        delay(time_step_ms);
        time_step_ms += 3; // Linear time step increase
    } while (!ADC_STAT_RDY_FILTER(stat_data));
    ret |= adc_reg_read24(dev, ADC_REG_DATA0 + channel, data);
    return ret;
}
