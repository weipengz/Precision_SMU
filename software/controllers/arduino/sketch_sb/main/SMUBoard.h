/*
SMUBoard.h - Firmware for Single SMU Board
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef SMUBOARD_h
#define SMUBOARD_h

#include "Globals.h"
#include "Controller.h"
#include "AD5753.h"
#include "MAX11254.h"
#include "MCP23S17.h"
#include "MC24AA02.h"


//board
spi_device get_mon_rst_dev();
spi_device get_mon_dev();
spi_device get_eeprom_dev();

exit_code board_mon_rst(void);
exit_code board_mon_setup(uint8_t opy);

exit_code board_read_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data);
exit_code board_write_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data);

exit_code board_is_exist(bool* exist);
exit_code board_setup(uint8_t* occupancy);

//channel
spi_device get_adc_dev(uint8_t ch);
spi_device get_dac_dev(uint8_t ch);

exit_code smu_output_switch(uint8_t ch, bool REFRES);
exit_code smu_output_enable(uint8_t ch, bool ENABLE);
exit_code smu_switch_output_target(uint8_t ch, bool REFRES);
exit_code smu_select_range(uint8_t ch, uint8_t range);
exit_code smu_set_dac_code(uint8_t ch, uint16_t dac_code);
exit_code smu_prepare_convert(uint8_t ch, bool pga, uint8_t gain);

exit_code smu_measure_i(uint8_t ch, uint32_t* adc_code);
exit_code smu_measure_v(uint8_t ch, uint32_t* adc_code);

exit_code smu_get_range(uint8_t ch, uint8_t* range);
exit_code smu_get_dac_code(uint8_t ch, uint16_t* dac_code);

exit_code smu_is_exist(uint8_t ch, bool* exist);
exit_code smu_setup(uint8_t ch);

#endif
