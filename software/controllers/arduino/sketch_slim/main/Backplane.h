/*
BackplaneRevA.h - Firmware for Backplane REV A
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef BACKPLANE_h
#define BACKPLANE_h

#include "Globals.h"

#define MAX11254_SPI_CLOCK    1000000
#define AD5753_SPI_CLOCK      1000000
#define MC25AA02_SPI_CLOCK    100000
#define MCP23S17_SPI_CLOCK    1000000

// get spi device
//bp
exit_code bpMonRst();
spi_device getBpMonDev(void);
//db
exit_code dbMonRst(uint8_t index);
spi_device getDbMonDev(uint8_t index);
spi_device getDbMonRstDev(uint8_t index);
spi_device getEEpromDev(uint8_t index);
//smu
spi_device getADCDev(uint8_t slot, uint8_t ch);
spi_device getDACDev(uint8_t slot, uint8_t ch);

// read and write reg
//mcp23s17
#define MCP23S17_REG_READ(reg_addr, hw_addr)   ((0x41 << 8) | ((reg_addr) & 0xFF) | (((hw_addr) & 0x7) << 9))
#define MCP23S17_REG_WRITE(reg_addr, hw_addr)  ((0x40 << 8) | ((reg_addr) & 0xFF) | (((hw_addr) & 0x7) << 9))
exit_code mcp23s17_write (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t reg_value);
exit_code mcp23s17_read (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_value);

//mc25aa02
#define MC25AA02_READ_CMD       0x03
#define MC25AA02_WRITE_CMD      0x02
#define MC25AA02_WRDI_CMD       0x04
#define MC25AA02_WREN_CMD       0x06
#define MC25AA02_RDSR_CMD       0x05
#define MC25AA02_WRSR_CMD       0x01
exit_code mc25aa02_read_status(spi_device dev, uint8_t* status);
exit_code mc25aa02_write_status(spi_device dev, uint8_t status);
exit_code mc25aa02_write(spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value);
exit_code mc25aa02_read(spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value);

//max11254
#define MAX11254_REG_READ(addr)     ((0xC0) | ((addr & 0x1F) << 1) | 0x1)
#define MAX11254_REG_WRITE(addr)    ((0xC0) | ((addr & 0x1F) << 1) | 0x0)
exit_code max11254_write (spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value);
exit_code max11254_read (spi_device dev, uint8_t reg_addr, uint8_t len, uint8_t* reg_value);
exit_code max11254_command (spi_device dev, uint8_t cmd);

//ad5753
#define AD5753_CRC8_POLY    0x07 // x^8 + x^2 + x^1 + x^0
#define AD5753_REG_WRITE(x)     ((0x80) | (x & 0x1F))
#define AD5753_REG_TWO_STAGE_READBACK_SELECT     0x13
#define AD5753_REG_NOP                           0x00
uint8_t ad5753_compute_crc8(uint8_t* data, uint8_t data_size);
exit_code transfer_ad5753_spi(uint8_t* buf, uint8_t* retbuf);
exit_code ad5753_write(spi_device dev, uint8_t reg_addr, uint16_t reg_value);
exit_code ad5753_read(spi_device dev, uint8_t reg_addr, uint16_t *reg_value);
exit_code ad5753_read32(spi_device dev, uint8_t reg_addr, uint32_t *reg_value);
exit_code ad5753_write_mask(spi_device dev, uint8_t reg_addr, uint32_t mask, uint16_t data);
#endif
