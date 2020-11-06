/*
Parser.h - Configures the socket server for the controller board
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef PARSER_h
#define PARSER_h

#include "Globals.h"
#include "Controller.h"
#include "SMUBoard.h"

//SCPI Commands
#define SCPI_IDN "IDN"
#define SCPI_RST "RST"
#define SCPI_SWP "SWP"
#define SCPI_CALC "CALC"

#define SCPI_CODE "CODE"
#define SCPI_CONF "CONF"
#define SCPI_MEAS "MEAS"
#define SCPI_SW   "SW"
#define SCPI_CALS "CALS"

#define MODE_VOLT "V"
#define MODE_CURR "I"

#define MEAS_VOLT "V"
#define MEAS_CURR "I"

#define SW_EXTERNAL  "EXT"
#define SW_INTERNAL  "INT"

class Parser
{
public:
    Parser(uint8_t* _occupancy);
    exit_code parsing(String cmd);
private:
    exit_code scpi_idn(void);
    exit_code scpi_rst(void);
    exit_code scpi_config(uint8_t ch, String range, bool isQuery);
    exit_code scpi_set_code(uint8_t ch, uint16_t value, bool isQuery);
    exit_code scpi_measure(uint8_t ch, String param);
    exit_code scpi_sweep_ch(uint8_t ch, String param);
    exit_code cmd_decomp(String cmd, String* header, String* value, uint8_t* ch, bool* isQuery);
    exit_code scpi_output_switch(uint8_t ch, String value, bool isQuery);
    exit_code scpi_calibrate_set(uint8_t ch, String value, bool isQuery);
    exit_code scpi_calibrate_rst(void);

    uint8_t* _occupancy;
};

#endif