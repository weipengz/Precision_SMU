/*
scpi_parser.h - scpi parser
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "Globals.h"

#define ADC_DEV     0
#define DAC_DEV     1
#define EEPROM_DEV  2
#define BPMON_DEV   3
#define DBMON_DEV   4
#define YUN_DEV     5

class Parser
{
public:
    exit_code parsing(String cmd);
private:
    exit_code cmd_decomp(String cmd, uint8_t* dev, uint8_t* slot, uint8_t* ch, String* param, bool* isRead, String* value);
};