/*
MONITOR.cxx
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once

#include "monitor.h"

spi_device get_bp_mon_dev()
{
	#define CS_NUM 1
    uint8_t cs_pin_list[CS_NUM] = {
        BSENABLE_N
    };
    uint8_t cs_pin_states[CS_NUM] = {
        HIGH
    };
    uint8_t cs_fast_pin = MONCS_N;
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

spi_device get_db_mon_dev(uint8_t slot)
{
	#define CS_NUM 9
    uint8_t cs_pin_list[CS_NUM] = {
        MONCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0
    };
    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        LOW,
        LOW,
        HIGH,
        BIT3_FILT(slot),
        BIT2_FILT(slot),
        BIT1_FILT(slot),
        BIT0_FILT(slot)
    };

    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;
    struct spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;
    #undef CS_NUM
    return ret;
}

spi_device get_db_mon_rst_dev(uint8_t slot)
{
	#define CS_NUM 9
    uint8_t cs_pin_list[CS_NUM] = {
        MONCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0
    };
    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        HIGH,
        HIGH,
        LOW,
        BIT3_FILT(slot),
        BIT2_FILT(slot),
        BIT1_FILT(slot),
        BIT0_FILT(slot)
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

exit_code monitor_reg_write (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t reg_value)
{
    ss_select(dev);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(MON_SPI_CLOCK_DIVIDER);
    bcm2835_spi_transfer(MON_REG_WRITE(hw_addr));
    bcm2835_spi_transfer(reg_addr);
    bcm2835_spi_transfer(reg_value);
    bcm2835_spi_end();
    ss_deselect(dev);
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI write: %02X%02X%02X\n",MON_REG_WRITE(hw_addr),reg_addr,reg_value);
    #endif

    return SUCCESS;	
}

exit_code monitor_reg_read (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_value)
{   
    ss_select(dev);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(MON_SPI_CLOCK_DIVIDER);
    bcm2835_spi_transfer(MON_REG_READ(hw_addr));
    bcm2835_spi_transfer(reg_addr);
    *reg_value = bcm2835_spi_transfer(0x00);
    bcm2835_spi_end();
    
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI write: %02X%02X\n",MON_REG_READ(hw_addr),reg_addr);
        printf("SPI read: %X\n",*reg_value);
    #endif
    
    ss_deselect(dev);
    return SUCCESS;	
}

exit_code monitor_read_all_regs(spi_device dev, uint8_t hw_addr, uint8_t* reg_values)
{
    exit_code ret = SUCCESS;
    uint8_t reg_i = 0;
    ss_preselect(dev);
    
    for (uint8_t reg_addr = 0x00; reg_addr <= 0x0A; reg_addr++)
    {
        ss_fast_select(dev);
        
        bcm2835_spi_begin();
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
        bcm2835_spi_setClockDivider(MON_SPI_CLOCK_DIVIDER);
        bcm2835_spi_transfer(MON_REG_READ(hw_addr));
        bcm2835_spi_transfer(reg_addr);
        *(reg_values+reg_i) = bcm2835_spi_transfer(0x00);
        bcm2835_spi_end();
        
        ss_deselect(dev);
        
        //~ #if STD_DEBUG and SPI_DEBUG
            //~ printf("SPI write: %02X%02X\n",MON_REG_READ(hw_addr),reg_addr);
            //~ printf("SPI read: %X\n",reg_values[reg_i]);
        //~ #endif
        
        reg_i++;
        ret |= SUCCESS;
    }
    for (uint8_t reg_addr = 0x10; reg_addr <= 0x1A; reg_addr++)
    {
        ss_fast_select(dev);
        
        bcm2835_spi_begin();
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
        bcm2835_spi_transfer(MON_REG_READ(hw_addr));
        bcm2835_spi_transfer(reg_addr);
        *(reg_values+reg_i) = bcm2835_spi_transfer(0x00);
        bcm2835_spi_end();
        
        ss_deselect(dev);
        
        //~ #if STD_DEBUG and SPI_DEBUG
            //~ printf("SPI write: %02X%02X\n",MON_REG_READ(hw_addr),reg_addr);
            //~ printf("SPI read: %X\n",reg_values[reg_i]);
        //~ #endif
        
        reg_i++;
        ret |= SUCCESS;
    }
    return ret;
}

exit_code monitor_iocon_cfg(spi_device dev, uint8_t hw_addr, uint8_t iocon_cfg)
{
	exit_code ret = SUCCESS;
    // Write IOCON defaults, in particular IOCON.BANK = 1
    ret |= monitor_reg_write(dev, hw_addr, 0x0A, iocon_cfg);
    // IOCON is now in registers 0x05 and 0x15
    return ret;
}

exit_code monitor_setup(spi_device dev, uint8_t hw_addr, uint8_t opy_a, uint8_t opy_b)
{
    exit_code ret = SUCCESS;
    // Write GPPUA and GPPUB to 0xFF, pull-up resistor on
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_GPPUA, 0xFF);
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_GPPUB, 0xFF);
    // Write DEFVAL to 0xFF (or read from EEPROM), expect 1
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_DEFVALA, 0xFF);
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_DEFVALB, 0xFF);
    // Write INTCON to 0xFF, compare with DEFVAL
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_INTCONA, 0xFF);
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_INTCONB, 0xFF);
    // Write GPINTEN to 0xFF (or read from EEPROM), enable interrupt functionality
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_GPINTENA, opy_a);
    ret |= monitor_reg_write(dev, hw_addr, MON_REG_GPINTENB, opy_b);
    return ret;
}

exit_code monitor_bp_reset()
{
    exit_code ret = SUCCESS;
    // Hardware reset the monitor chips.
    digitalWrite(MONRST_N,  LOW);
    delayMicroseconds(1);
    digitalWrite(MONRST_N,  HIGH);
    delayMicroseconds(2);
    #if STD_DEBUG
        printf("----   Monitor Reset   ----\n");
    #endif
    return ret;
}

exit_code monitor_db_reset(uint8_t slot)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_db_mon_rst_dev(slot);
    ss_select(dev);
    delayMicroseconds(1);
    ss_deselect(dev);
    delayMicroseconds(2);
    //~ #if STD_DEBUG
        //~ printf("---- DB(%d) Mon. Reset ----\n",slot);
    //~ #endif
    return ret;
}
