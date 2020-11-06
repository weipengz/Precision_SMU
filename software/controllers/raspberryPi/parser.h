/*
parser.h - Parser
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "globals.h"
#include <string>
#include <cstring>

using namespace std;

typedef uint16_t scpi_cmd;
//SCPI Commands
#define SCPI_IDN  "IDN?"
#define SCPI_SHDN "SHDN"
#define SCPI_RST  "RST"
#define SCPI_CAL  "CAL"
#define SCPI_CALC "CALC"
#define SCPI_OCC  "OCC?"

#define SCPI_CODE "CODE"
#define SCPI_CURR "CURR"
#define SCPI_VOLT "VOLT"
#define SCPI_CONF "CONF"
#define SCPI_MEAS "MEAS"
#define SCPI_REFR "REFR"
#define SCPI_CALS "CALS"

#define DAC_REFIN           "AD_REFIN"
#define DAC_REFGND          "AD_REFGND"
#define DAC_ISENSE          "AD_CURR"
#define DAC_VLOAD           "AD_VLOAD"
#define ADC_CURRENT         "MAX_CURR"
#define ADC_SENSE_VOLTAGE   "MAX_VSENSE"
#define ADC_LOAD_VOLTAGE    "MAX_VLOAD"

#define RANGE_5V_UNI        "5V_UNI"
#define RANGE_10V_UNI       "10V_UNI"
#define RANGE_5V_BIP        "5V_BIP"
#define RANGE_10V_BIP       "10V_BIP"
#define RANGE_20mA_UNI      "20mA_UNI"
#define RANGE_24mA_UNI      "24mA_UNI"
#define RANGE_4_24mA        "4_24mA"
#define RANGE_20mA_BIP      "20mA_BIP"
#define RANGE_24mA_BIP      "24mA_BIP"
#define RANGE_22mA_OVR      "22mA_OVR"

#define FOLDER_UNCAL "./calibration/uncalibrated/"
#define FOLDER_CAL   "./calibration/calibrated/"

exit_code convert_value_to_code(double value, uint8_t range, uint16_t* dac_code);
exit_code convert_code_to_value(double* value, string type, uint32_t dac_code);

class Parser
{
public:
    Parser(uint8_t* _occupancy);
    exit_code parsing(string cmd);
private:
    exit_code cmd_decomp(string cmd, string* header, string* value, uint8_t* slot, uint8_t* ch, bool* isQuery);
    
    exit_code scpi_idn(void);
    exit_code scpi_shdn(void);
    exit_code scpi_rst(void);
    
    exit_code scpi_local_cal(void);
    exit_code cal_sweep(string folder, uint8_t n_rep, uint16_t cal_step);
    exit_code calc_cal_params(uint8_t n_rep, uint16_t cal_step);
    exit_code scpi_cal_rst(void);
    exit_code scpi_set_cal(uint8_t slot, uint8_t ch, string value, bool isQuery);
    
    exit_code scpi_occ(void);
    exit_code scpi_config(uint8_t slot, uint8_t ch, string range, bool isQuery);
    exit_code scpi_dac_code(uint8_t slot, uint8_t ch, string value, bool isQuery);
    
    exit_code scpi_current(uint8_t slot, uint8_t ch, string value, bool isQuery);
    exit_code scpi_voltage(uint8_t slot, uint8_t ch, string value, bool isQuery);
    
    exit_code scpi_measure(uint8_t slot, uint8_t ch, string type);
    exit_code scpi_output_switch(uint8_t slot, uint8_t ch, string refr, bool isQuery);

    uint8_t* _occupancy;
};

