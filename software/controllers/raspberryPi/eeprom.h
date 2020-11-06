/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef EEPROM_h
#define EEPROM_h

#include "globals.h"

#define EEPROM_SPI_CLOCK_DIVIDER      BCM2835_SPI_CLOCK_DIVIDER_512

#define EEPROM_READ_CMD       0x03
#define EEPROM_WRITE_CMD      0x02
#define EEPROM_WRDI_CMD       0x04
#define EEPROM_WREN_CMD       0x06
#define EEPROM_RDSR_CMD       0x05
#define EEPROM_WRSR_CMD       0x01

spi_device get_eeprom_dev(uint8_t slot);

exit_code eeprom_read_status(spi_device dev, uint8_t* status);

exit_code eeprom_read(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out);
exit_code eeprom_write(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in);

exit_code _eeprom_read(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_out);
exit_code _eeprom_write(spi_device dev, uint8_t addr, uint8_t n_bytes, uint8_t* mem_in);
#endif
