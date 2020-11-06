/*
BackplaneRevA.h - Firmware for Backplane REV A
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "Backplane.h"

SPISettings max11254_spi_settings(MAX11254_SPI_CLOCK, MSBFIRST, SPI_MODE0);
SPISettings ad5753_spi_settings(AD5753_SPI_CLOCK, MSBFIRST, SPI_MODE1);
SPISettings mcp23s17_spi_settings(MCP23S17_SPI_CLOCK, MSBFIRST, SPI_MODE0);
SPISettings MC25AA02_spi_settings(MC25AA02_SPI_CLOCK, MSBFIRST, SPI_MODE0);

//backplane
exit_code bpMonRst()
{
    exit_code ret = SUCCESS;
    // Hardware reset the monitor chips.
    digitalWrite(MONITORRST_N,  LOW);
    delayMicroseconds(1);
    digitalWrite(MONITORRST_N,  HIGH);
    delayMicroseconds(2);
    return ret;
}

spi_device getBpMonDev()
{
    #define CS_NUM 1
    uint8_t cs_pin_list[CS_NUM] = {
        BSENABLE_N
    };
    uint8_t cs_pin_states[CS_NUM] = {
        HIGH
    };
    uint8_t cs_fast_pin = MONITORCS_N;
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

//daughterboard
exit_code dbMonRst(uint8_t index)
{
    exit_code ret = SUCCESS;
    spi_device dev = getDbMonRstDev(index);
    ss_select(dev);
    ss_deselect(dev);
    return ret;
}

spi_device getDbMonRstDev(uint8_t index)
{
    #define CS_NUM 9
    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
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
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
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

spi_device getDbMonDev(uint8_t index)
{
    #define CS_NUM 9
    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
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
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
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

spi_device getEEpromDev(uint8_t index)
{
    #define CS_NUM 9
    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
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
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
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

//smu
spi_device getADCDev(uint8_t slot, uint8_t ch)
{
    #define CS_NUM 10
    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
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

spi_device getDACDev(uint8_t slot, uint8_t ch)
{
    #define CS_NUM 10
    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
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

//mcp23s17
exit_code mcp23s17_write (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(mcp23s17_spi_settings);
    SPI.transfer16(MCP23S17_REG_WRITE(reg_addr, hw_addr));
    SPI.transfer(reg_value);
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code mcp23s17_read (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(mcp23s17_spi_settings);
    SPI.transfer16(MCP23S17_REG_READ(reg_addr, hw_addr));
    *reg_value = SPI.transfer(0x00);
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

//mc25aa02
exit_code mc25aa02_read_status(spi_device dev, uint8_t* status)
{
    ss_select(dev);
    SPI.beginTransaction(MC25AA02_spi_settings);
    SPI.transfer(MC25AA02_RDSR_CMD);
    *status = SPI.transfer(0x00);
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code mc25aa02_write_status(spi_device dev, uint8_t status)
{
    ss_select(dev);
    SPI.beginTransaction(MC25AA02_spi_settings);
    SPI.transfer(MC25AA02_WRSR_CMD);
    SPI.transfer(status);
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code mc25aa02_write(spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value)
{
    SPI.beginTransaction(MC25AA02_spi_settings);
    delayMicroseconds(100);
    ss_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC25AA02_WREN_CMD);
    ss_deselect(dev);
    delayMicroseconds(100);
    ss_fast_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC25AA02_WRITE_CMD);
    SPI.transfer(reg_addr);
    for (uint8_t i = 0; i < len; i++)
    {
        SPI.transfer(reg_value[i]);
    }
    delayMicroseconds(100);
    ss_deselect(dev);
    delayMicroseconds(100);
    SPI.endTransaction();
    uint8_t estatus;
    do
    {
        mc25aa02_read_status(dev, &estatus);
        // Serial.println(estatus,DEC);
        delayMicroseconds(500);
    }
    while ( (estatus&0x01)==0x01 );
    return SUCCESS;
}

exit_code mc25aa02_read(spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value)
{
    SPI.beginTransaction(MC25AA02_spi_settings);
    delayMicroseconds(100);
    ss_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC25AA02_READ_CMD);
    SPI.transfer(reg_addr);
    for (uint8_t i = 0; i < len; i++)
    {
        reg_value[i] = SPI.transfer(0x00);
    }
    delayMicroseconds(100);
    ss_deselect(dev);
    delayMicroseconds(100);
    SPI.endTransaction();
    return SUCCESS;
}

//max11254
exit_code max11254_write (spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(MAX11254_REG_WRITE(reg_addr));
    for(uint8_t i=0; i < len; i++){
        SPI.transfer(reg_value[i]);
    }
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code max11254_read (spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(MAX11254_REG_READ(reg_addr));
    for(int i=0; i < len; i++){
        reg_value[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

exit_code max11254_command (spi_device dev, uint8_t cmd)
{
    ss_fast_select(dev);
    SPI.beginTransaction(max11254_spi_settings);
    SPI.transfer(cmd);
    SPI.endTransaction();
    ss_deselect(dev);
    return SUCCESS;
}

// ad5753
uint8_t ad5753_compute_crc8(uint8_t* data, uint8_t data_size)
{
    uint8_t i;
    uint8_t crc = 0;
    while (data_size) {
        for (i = 0x80; i != 0; i >>= 1) {
            if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
                crc <<= 1;
                crc ^= AD5753_CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
        data++;
        data_size--;
    }
    return crc;
}

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

exit_code ad5753_write(spi_device dev, uint8_t reg_addr, uint16_t reg_value)
{
    exit_code ret = SUCCESS;
    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(reg_addr);
    buf[1] = (reg_value >> 8);
    buf[2] = (reg_value & 0xFF);
    ss_select(dev);
    uint8_t retbuf[4];
    transfer_ad5753_spi(buf, retbuf);
    ss_deselect(dev);
    return ret;
}

exit_code ad5753_read(spi_device dev, uint8_t reg_addr, uint16_t *reg_value)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(AD5753_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x0;
    buf[2] = (reg_addr & 0x1F);
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
    *reg_value = (retbuf[1] << 8) | retbuf[2];
    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1));  // check slip bit
    bool fault = ((retbuf[0] >> 5) & 0x1);  // get fault pin status
    ret |= spi_fault * SPI_FAILURE;
    ret |= fault * AD5753_FAULT;
    return ret;
}

exit_code ad5753_read32(spi_device dev, uint8_t reg_addr, uint32_t *reg_value)
{
    exit_code ret = SUCCESS;
    // Activate TWO_STAGE_READBACK_SELECT
    uint8_t buf[4];
    buf[0] = AD5753_REG_WRITE(AD5753_REG_TWO_STAGE_READBACK_SELECT);
    buf[1] = 0x0;
    buf[2] = (reg_addr & 0x1F);
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
    *reg_value = retbuf[0];
    *reg_value <<= 8;
    *reg_value |= retbuf[1];
    *reg_value <<= 8;
    *reg_value |= retbuf[2];
    bool spi_fault = !(((retbuf[0] >> 7) & 0x1) ^ ((retbuf[0] >> 6) & 0x1)); 
    bool fault = ((retbuf[0] >> 5) & 0x1); 
    ret |= spi_fault * SPI_FAILURE;
    ret |= fault * AD5753_FAULT;
    return ret;
}

exit_code ad5753_write_mask(spi_device dev, uint8_t reg_addr, uint32_t mask, uint16_t data)
{
    uint16_t reg_data;
    exit_code ret = SUCCESS;
    ret |= ad5753_read(dev, reg_addr, &reg_data);
    reg_data &= ~mask;
    reg_data |= data;
    ret |= ad5753_write(dev, reg_addr, reg_data);
    return ret;
}