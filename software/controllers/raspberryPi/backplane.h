/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef BACKPLANE_h
#define BACKPLANE_h

#include "globals.h"

//Global
exit_code imebuild_setup(uint8_t* occupancy);


//Backplane
exit_code backplane_setup(uint16_t opy_db);


//Daughterboard
exit_code daughterboard_is_exist(uint8_t slot, bool* db_is_exist);
exit_code daughterboard_setup(uint8_t slot, uint8_t opy_ch);
exit_code daughterboard_monitor_diag(uint8_t slot, uint8_t* adc_fault, uint8_t* power_fault);


//SMU
exit_code smu_is_exist(uint8_t slot, uint8_t ch, bool* exist);
exit_code smu_setup(uint8_t slot, uint8_t ch);
exit_code smu_startup_sequence(void);
exit_code smu_output_enable(uint8_t slot, uint8_t ch, bool ENABLE);
exit_code smu_reference_switch(uint8_t slot, uint8_t ch, bool REFRES);
exit_code smu_output_target(uint8_t slot, uint8_t ch, bool REFRES);
exit_code smu_select_range(uint8_t slot, uint8_t ch, uint8_t range);
exit_code smu_get_range(uint8_t slot, uint8_t ch, uint8_t* range);
exit_code smu_set_dac_code(uint8_t slot, uint8_t ch, uint16_t dac_code);
exit_code smu_get_dac_code(uint8_t slot, uint8_t ch, uint16_t* dac_code);
exit_code smu_measure_v_sense(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info);
exit_code smu_measure_v_load(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info);
exit_code smu_measure_i(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info);
exit_code smu_read_debug_adc(uint8_t slot, uint8_t ch, uint8_t ip_addr, uint16_t* adc_data);
#endif
