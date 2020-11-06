/*
MCP23S17.h - MCP23S17 Expander functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include <SPI.h>
#include "MCP23S17.h"

SPISettings mcp23s17_spi_settings (MCP23S17_SPI_CLOCK, MSBFIRST, SPI_MODE0);

exit_code mcp23s17_reg_write(spi_device dev, uint8_t hw_addr, uint8_t reg_address, uint8_t reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(mcp23s17_spi_settings);
    SPI.transfer16(MCP23S17_REG_WRITE(reg_address, hw_addr));
    SPI.transfer(reg_value);
    SPI.endTransaction();
    ss_deselect(dev);
    #if SERIAL_DEBUG and SPI_DEBUG
        Serial.print("SPI write: ");
        Serial.print(MCP23S17_REG_WRITE(reg_address, hw_addr), HEX);
        Serial.println(reg_value, HEX);
    #endif
    return SUCCESS;
}

exit_code mcp23s17_reg_read(spi_device dev, uint8_t hw_addr, uint8_t reg_address, uint8_t* reg_value)
{
    ss_select(dev);
    SPI.beginTransaction(mcp23s17_spi_settings);
    SPI.transfer16(MCP23S17_REG_READ(reg_address, hw_addr));
    *reg_value = SPI.transfer(0x00);
    SPI.endTransaction();
    ss_deselect(dev);
    #if SERIAL_DEBUG and SPI_DEBUG
        Serial.print("SPI write: ");
        Serial.println(MCP23S17_REG_READ(reg_address, hw_addr), HEX);
        Serial.print("SPI read: ");
        Serial.println(*reg_value, HEX);
    #endif
    return SUCCESS;
}

exit_code mcp23s17_read_all_regs (spi_device dev, uint8_t hw_addr, uint8_t* reg_values)
{
    exit_code ret = SUCCESS;
    uint8_t reg_i = 0;
    ss_preselect(dev);

    for (uint8_t reg_address = 0x00; reg_address <= 0x0A; reg_address++)
    {
        ss_fast_select(dev);
        SPI.beginTransaction(mcp23s17_spi_settings);
        SPI.transfer16(MCP23S17_REG_READ(reg_address, hw_addr));
        reg_values[reg_i] = SPI.transfer(0x00);
        SPI.endTransaction();
        ss_deselect(dev);
        #if SERIAL_DEBUG and SPI_DEBUG
            Serial.print("SPI write: ");
            Serial.println(MCP23S17_REG_READ(reg_address, hw_addr), HEX);
            Serial.print("SPI read: ");
            Serial.println(reg_values[reg_i], HEX);
        #endif
        reg_i++;
        ret |= SUCCESS;
    }

    for (uint8_t reg_address = 0x10; reg_address <= 0x1A; reg_address++)
    {
        ss_fast_select(dev);
        SPI.beginTransaction(mcp23s17_spi_settings);
        SPI.transfer16(MCP23S17_REG_READ(reg_address, hw_addr));
        reg_values[reg_i] = SPI.transfer(0x00);
        SPI.endTransaction();
        ss_deselect(dev);
        #if SERIAL_DEBUG and SPI_DEBUG
            Serial.print("SPI write: ");
            Serial.println(MCP23S17_REG_READ(reg_address, hw_addr), HEX);
            Serial.print("SPI read: ");
            Serial.println(reg_values[reg_i], HEX);
        #endif
        reg_i++;
        ret |= SUCCESS;
    }

    return ret;
}


exit_code mcp23s17_iocon_cfg(spi_device dev, uint8_t hw_addr, uint8_t iocon_cfg)
{
    exit_code ret = SUCCESS;

    // Write IOCON defaults, in particular IOCON.BANK = 1
    ret |= mcp23s17_reg_write(dev, hw_addr, 0x0A, iocon_cfg);
    // IOCON is now in registers 0x05 and 0x15

    return ret;
}

exit_code mcp23s17_setup(spi_device dev, uint8_t hw_addr, uint8_t opy_a, uint8_t opy_b)
{
    exit_code ret = SUCCESS;

    // Write GPPUA and GPPUB to 0xFF, pull-up resistor on
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_GPPUA, 0xFF);
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_GPPUB, 0xFF);

    // Write DEFVAL to 0xFF (or read from EEPROM), expect 1
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_DEFVALA, 0xFF);
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_DEFVALB, 0xFF);

    // Write INTCON to 0xFF, compare with DEFVAL
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_INTCONA, 0xFF);
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_INTCONB, 0xFF);

    // Write GPINTEN to 0xFF (or read from EEPROM), enable interrupt functionality
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_GPINTENA, opy_a);//0xFF
    ret |= mcp23s17_reg_write(dev, hw_addr, MCP23S17_REG_GPINTENB, opy_b);//0xFF

    return ret;
}
