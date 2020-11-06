/*
MCP23S17.h - MCP23S17 Expander functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef MCP23S17_h
#define MCP23S17_h

#include <SPI.h>
#include "Globals.h"

//Register map (pg 16, BANK=1)

#define MCP23S17_REG_IODIRA     0x00
#define MCP23S17_REG_IOPOLA     0x01
#define MCP23S17_REG_GPINTENA   0x02
#define MCP23S17_REG_DEFVALA    0x03
#define MCP23S17_REG_INTCONA    0x04
#define MCP23S17_REG_IOCONA     0x05
#define MCP23S17_REG_GPPUA      0x06
#define MCP23S17_REG_INTFA      0x07
#define MCP23S17_REG_INTCAPA    0x08
#define MCP23S17_REG_GPIOA      0x09
#define MCP23S17_REG_OLATA      0x0A

#define MCP23S17_REG_IODIRB     0x10
#define MCP23S17_REG_IOPOLB     0x11
#define MCP23S17_REG_GPINTENB   0x12
#define MCP23S17_REG_DEFVALB    0x13
#define MCP23S17_REG_INTCONB    0x14
#define MCP23S17_REG_IOCONB     0x15
#define MCP23S17_REG_GPPUB      0x16
#define MCP23S17_REG_INTFB      0x17
#define MCP23S17_REG_INTCAPB    0x18
#define MCP23S17_REG_GPIOB      0x19
#define MCP23S17_REG_OLATB      0x1A

#define MCP23S17_REG_READ(reg_addr, hw_addr)         ((0x41 << 8) | ((reg_addr) & 0xFF) | (((hw_addr) & 0x7) << 9))
#define MCP23S17_REG_WRITE(reg_addr, hw_addr)        ((0x40 << 8) | ((reg_addr) & 0xFF) | (((hw_addr) & 0x7) << 9))

#define MCP23S17_SPI_CLOCK      1000000

exit_code mcp23s17_read_all_regs (spi_device dev, uint8_t hw_addr, uint8_t* reg_values);
exit_code mcp23s17_reg_write (spi_device dev, uint8_t hw_addr, uint8_t reg_address, uint8_t reg_value);
exit_code mcp23s17_reg_read (spi_device dev, uint8_t hw_addr, uint8_t reg_address, uint8_t* reg_value);

exit_code mcp23s17_iocon_cfg(spi_device dev, uint8_t hw_addr, uint8_t iocon_cfg);
exit_code mcp23s17_setup(spi_device dev, uint8_t hw_addr, uint8_t opy_a, uint8_t opy_b);

#endif
