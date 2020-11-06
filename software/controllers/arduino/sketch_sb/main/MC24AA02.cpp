/*
MC24AA02.h - MC24AA02 EEPROM functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "MC24AA02.h"

SPISettings MC25AA02_spi_settings (MC24AA02_SPI_CLOCK, MSBFIRST, SPI_MODE0);

exit_code mc24aa02_read_status(spi_device dev, uint8_t* status)
{
    ss_select(dev);
    SPI.beginTransaction(MC25AA02_spi_settings);
    SPI.transfer(MC24AA02_RDSR_CMD);
    *status = SPI.transfer(0x00);
    SPI.endTransaction();
    ss_deselect(dev);

    return SUCCESS;
}

exit_code mc24aa02_read_mem_addr(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out)
{
    SPI.beginTransaction(MC25AA02_spi_settings);
    delayMicroseconds(100);
    ss_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC24AA02_READ_CMD);
    SPI.transfer(addr);
    for (uint8_t i = 0; i < n_bytes; i++)
    {
        mem_out[i] = SPI.transfer(0x00);
    }
    // SPI.transfer(mem_out, n_bytes);
    delayMicroseconds(100);
    ss_deselect(dev);
    delayMicroseconds(100);
    SPI.endTransaction();

    return SUCCESS;
}

exit_code mc24aa02_write_mem_addr(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in)
{
    SPI.beginTransaction(MC25AA02_spi_settings);
    delayMicroseconds(100);
    ss_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC24AA02_WREN_CMD);
    ss_deselect(dev);
    delayMicroseconds(100);
    ss_fast_select(dev);
    delayMicroseconds(100);
    SPI.transfer(MC24AA02_WRITE_CMD);
    SPI.transfer(addr);
    for (uint8_t i = 0; i < n_bytes; i++)
    {
        SPI.transfer(mem_in[i]);
    }
    // SPI.transfer(mem_in, n_bytes);
    
    delayMicroseconds(100);
    ss_deselect(dev);
    delayMicroseconds(100);
    SPI.endTransaction();

    return SUCCESS;
}