/*
BackplaneRevA.h - Firmware for Backplane REV A
Copyright (c) 2019 Lightwave Lab, Princeton University
*/


#ifndef BACKPLANE_REV_A_h
#define BACKPLANE_REV_A_h

#include <Arduino.h>
#include "ArduinoYun2Controller.h"
#include "Globals.h"
#include "AD5753.h"
#include "MAX11254.h"
#include "MCP23S17.h"
#include "MC24AA02.h"

#define FAULTMON_ADDR       0x0
#define PGOODMON_ADDR       0x1

#define BIT0_FILT(x)         (uint8_t) (((x) & BIT(0)) >> 0)
#define BIT1_FILT(x)         (uint8_t) (((x) & BIT(1)) >> 1)
#define BIT2_FILT(x)         (uint8_t) (((x) & BIT(2)) >> 2)
#define BIT3_FILT(x)         (uint8_t) (((x) & BIT(3)) >> 3)


class Backplane
{
public:
    exit_code setup(uint16_t opy_db);
    exit_code monitor_reset(void);
    exit_code monitor_diag(uint16_t* dac_fault, uint16_t* power_fault);
    exit_code scan_all_slots(uint8_t* occupancy);

    exit_code print_all_mon_regs(void);

private:
    spi_device get_mon_spi_dev(void);
};

class Daughterboard
{
public:
    Daughterboard(uint8_t index);
    // Daughterboard(char* serial_number);

    exit_code setup(uint8_t opy_ch);
    exit_code read_mac_address(uint8_t* mac_addr);
    // exit_code read_eeprom_status(uint8_t* estatus);
    exit_code read_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data);
    exit_code write_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data);
    exit_code monitor_reset(void);
    exit_code monitor_diag(uint8_t* dac_fault, uint8_t* power_fault);

    exit_code isExist(bool* exist);

    exit_code print_all_slots(void);
    exit_code print_mac_address(void);
    exit_code flicker_eeprom_spi_dev(void);

    exit_code print_all_mon_regs(void);

    uint8_t index;

private:
    spi_device get_mon_spi_dev(void);
    spi_device get_monitor_reset_spi_dev(void);
    spi_device get_eeprom_spi_dev(void);
};

#define SMU_CURRENT_MODE        0x0
#define SMU_VOLTAGE_MODE        0x1

#define SMU_DEFAULT_RANGE       AD5753_DAC_CONFIG_RANGE_5V_UNI

#define MEAS_CURRENT        true
#define MEAS_VOLTAGE        false

class SMU
{
public:
    // Basic functions
    SMU(Daughterboard* db, uint8_t channel);
    SMU(Daughterboard* db, uint8_t channel, uint32_t* calInfo);
    exit_code startup_sequence(void);
    exit_code output_switch(bool ENABLE);
    exit_code toggle_reference_switch(bool REFRES);
    exit_code switch_output_target(bool REFRES);
    // exit_code adc_hw_reset(void);

    // Source Functions
    exit_code select_current_mode(uint8_t range);
    exit_code select_voltage_mode(uint8_t range);
    exit_code set_current(uint16_t dac_code);
    exit_code set_voltage(uint16_t dac_code);

    // Measure Functions
    exit_code select_range(bool pga, uint8_t gain);
    exit_code measure_current(uint32_t* adc_code);
    exit_code measure_sense_voltage(uint32_t* adc_code);
    exit_code measure_load_voltage(uint32_t* adc_code);
    exit_code measure_source_current(uint32_t* adc_code);

    //The following two are available only if the jumpers on board are shorted
    exit_code read_debug_adc1(uint16_t* adc_data);
    exit_code read_debug_adc2(uint16_t* adc_data);

    exit_code read_debug_adc(uint8_t ip_address, uint16_t* adc_data);

    exit_code get_range_mode(uint8_t* range, uint8_t* dac_mode);
    exit_code get_dac_code(uint16_t* dac_code);

    exit_code diagonse(void);

    exit_code isExist(bool* exist);

private:
    spi_device get_spi_device(bool dadcs);
    spi_device get_dac_spi_dev(void);
    spi_device get_adc_spi_dev(void);

    Daughterboard* _db;
    uint8_t _channel;
    uint8_t _dac_mode;
    uint8_t _range;

    uint32_t _gain_i;
    uint32_t _offset_i;
    uint32_t _gain_v;
    uint32_t _offset_v;
};

#endif
