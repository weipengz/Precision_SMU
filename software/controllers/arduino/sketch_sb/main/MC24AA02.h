/*
MC24AA02.h - MC24AA02 EEPROM functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef MC24AA02_h
#define MC24AA02_h

#include <SPI.h>
#include "Globals.h"

#define MC24AA02_SPI_CLOCK      100000//1000000

#define MC24AA02_READ_CMD       0x03
#define MC24AA02_WRITE_CMD      0x02
#define MC24AA02_WRDI_CMD       0x04
#define MC24AA02_WREN_CMD       0x06
#define MC24AA02_RDSR_CMD       0x05
#define MC24AA02_WRSR_CMD       0x01


exit_code mc24aa02_read_status(spi_device dev, uint8_t* status);
exit_code mc24aa02_read_mem_addr(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out);
exit_code mc24aa02_write_mem_addr(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in);

#endif
