/*
MONITOR.h
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef MONITOR_h
#define MONITOR_h

#include "globals.h"

//Register map (pg 16, BANK=1)
#define MON_REG_IODIRA     0x00
#define MON_REG_IOPOLA     0x01
#define MON_REG_GPINTENA   0x02
#define MON_REG_DEFVALA    0x03
#define MON_REG_INTCONA    0x04
#define MON_REG_IOCONA     0x05
#define MON_REG_GPPUA      0x06
#define MON_REG_INTFA      0x07
#define MON_REG_INTCAPA    0x08
#define MON_REG_GPIOA      0x09
#define MON_REG_OLATA      0x0A

#define MON_REG_IODIRB     0x10
#define MON_REG_IOPOLB     0x11
#define MON_REG_GPINTENB   0x12
#define MON_REG_DEFVALB    0x13
#define MON_REG_INTCONB    0x14
#define MON_REG_IOCONB     0x15
#define MON_REG_GPPUB      0x16
#define MON_REG_INTFB      0x17
#define MON_REG_INTCAPB    0x18
#define MON_REG_GPIOB      0x19
#define MON_REG_OLATB      0x1A

#define MON_REG_READ(x)   (0x41 | (((x)&0x7)<<1))
#define MON_REG_WRITE(x)  (0x40 | (((x)&0x7)<<1))

#define MON_SPI_CLOCK 1000000
#define MON_SPI_CLOCK_DIVIDER    BCM2835_SPI_CLOCK_DIVIDER_512

spi_device get_bp_mon_dev();
spi_device get_db_mon_dev(uint8_t slot);
spi_device get_db_mon_rst_dev(uint8_t slot);

exit_code monitor_reg_write (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t reg_value);
exit_code monitor_reg_read (spi_device dev, uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_value);
exit_code monitor_read_all_regs (spi_device dev, uint8_t hw_addr, uint8_t* reg_values);

exit_code monitor_iocon_cfg(spi_device dev, uint8_t hw_addr, uint8_t iocon_cfg);
exit_code monitor_setup(spi_device dev, uint8_t hw_addr, uint8_t opy_a, uint8_t opy_b);

exit_code monitor_bp_reset();
exit_code monitor_db_reset(uint8_t slot);
#endif
