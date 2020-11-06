/*
eeprom.cxx - EEPROM functionality
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "eeprom.h"

spi_device get_eeprom_dev(uint8_t slot)
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

exit_code eeprom_read_status(spi_device dev, uint8_t* status)
{
    ss_select(dev);
    //~ wiringPiSPISetup(0, EEPROM_SPI_CLOCK);
    //~ uint8_t cmd = EEPROM_RDSR_CMD;
    //~ wiringPiSPIDataRW(0, &cmd, 1);
    //~ wiringPiSPIDataRW(0, status, 1);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(EEPROM_SPI_CLOCK_DIVIDER);
    uint8_t cmd = EEPROM_RDSR_CMD;
    bcm2835_spi_transfer(cmd);
    *status = bcm2835_spi_transfer(0x00);
    bcm2835_spi_end();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code eeprom_read(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out)
{
    bool isGood;
    uint8_t _data1[n_bytes];
    uint8_t _data2[n_bytes];
    uint8_t _data3[n_bytes];
    do
    {
        isGood = true;
        _eeprom_read(dev,addr,n_bytes,_data1);
        _eeprom_read(dev,addr,n_bytes,_data2);
        _eeprom_read(dev,addr,n_bytes,_data3);
        for (uint8_t idx = 0; idx < n_bytes; idx++)
        {
            if (_data1[idx]!=_data2[idx]) isGood = false;
            if (_data1[idx]!=_data3[idx]) isGood = false;
        }
    } while (!isGood);
    for (uint8_t idx = 0; idx < n_bytes; idx++) mem_out[idx]=_data1[idx];
    return SUCCESS;
}

exit_code eeprom_write(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in)
{
    bool isGood;
    uint8_t data[n_bytes];
    do
    {
        isGood = true;
        _eeprom_write(dev,addr,n_bytes,mem_in);
        eeprom_read(dev,addr,n_bytes,data);
        for (uint8_t idx = 0; idx < n_bytes; idx++) 
            if (data[idx]!=mem_in[idx]) isGood = false;
    } while (!isGood);
    return SUCCESS;
}

exit_code _eeprom_read(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out)
{
    uint8_t cmd = EEPROM_READ_CMD;
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(EEPROM_SPI_CLOCK_DIVIDER);
    
    ss_select(dev);
    bcm2835_spi_transfer(cmd);
    bcm2835_spi_transfer(addr);
    for (int i = 0; i < n_bytes; i++)
    {
        mem_out[i] = bcm2835_spi_transfer(0x00);
    }
    ss_deselect(dev);
    
    #if STD_DEBUG and SPI_DEBUG
        printf("SPI write: %X%X\n",EEPROM_READ_CMD,addr);
        printf("SPI read: ");
        for (int i = 0; i < n_bytes; i++)
        {
            printf("%X.",mem_out[i]);
        }
        printf("\n");
    #endif
    
    return SUCCESS;
}

exit_code _eeprom_write(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in)
{
    uint8_t cmd;
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(EEPROM_SPI_CLOCK_DIVIDER);
    //~ delayMicroseconds(1);
    
    ss_select(dev);
    cmd = EEPROM_WREN_CMD;
    bcm2835_spi_transfer(cmd);
    ss_deselect(dev);
    
    //~ delayMicroseconds(1);
    
    ss_fast_select(dev);
    cmd = EEPROM_WRITE_CMD;
    bcm2835_spi_transfer(cmd);
    bcm2835_spi_transfer(addr);
    for (int i = 0; i < n_bytes; i++)
    {
        bcm2835_spi_transfer(mem_in[i]);
    }
    //~ delayMicroseconds(1);
    ss_deselect(dev);
    
    uint8_t status;
    do
    {
        eeprom_read_status(dev, &status);
        //~ printf("%d\n",status);
        delayMicroseconds(100);
    }
    while( (status&0x01)==0x01 );
    return SUCCESS;
}
