#include <arduino.h>
#include "Globals.h"
#include "BackplaneRevA.h"
#include "ControllerLib.h"

// Enable SCPI debug
#define SCPI_DEBUG flase

typedef uint16_t scpi_cmd;
//SCPI Commands
#define SCPI_IDN "IDN?"
#define SCPI_RST "RST"
#define SCPI_CAL "CAL"
// #define SCPI_CALQ "CAL?"
#define SCPI_OCC "OCC?"
#define SCPI_CALC "CALC"

#define SCPI_CURR "CURR"
#define SCPI_VOLT "VOLT"
#define SCPI_CONF "CONF"
#define SCPI_MEAS "MEAS"
#define SCPI_REFR "REFR"
#define SCPI_CALS "CALS"

#define AD5753_REFIN           "AD_REFIN"
#define AD5753_REFGND          "AD_REFGND"
#define AD5753_ISENSE          "AD_CURR"
#define AD5753_LOADP           "AD_VLOAD"
#define MAX11254_CURRENT       "MAX_CURR"
#define MAX11254_SENSE_VOLTAGE "MAX_VSENSE"
#define MAX11254_LOAD_VOLTAGE  "MAX_VLOAD"

#define RANGE_5V_UNI          "5V_UNI"
#define RANGE_10V_UNI         "10V_UNI"
#define RANGE_5V_BIP          "5V_BIP"
#define RANGE_10V_BIP         "10V_BIP"
#define RANGE_20mA_UNI        "20mA_UNI"
#define RANGE_24mA_UNI        "24mA_UNI"
#define RANGE_4_24mA          "4_24mA"
#define RANGE_20mA_BIP        "20mA_BIP"
#define RANGE_24mA_BIP        "24mA_BIP"
#define RANGE_22mA_OVR        "22mA_OVR"

#define SW_EXTERNAL  "EXTERNAL"
#define SW_INTERNAL  "INTERNAL"

class Parser
{
public:
    Parser(Controller* controller, Backplane* backplane, uint8_t* _occupancy);
    exit_code parsing(String cmd);
private:
    exit_code scpi_idn(void);
    exit_code scpi_rst(void);
    exit_code scpi_cal(void);
    exit_code scpi_occ(void);
    exit_code scpi_config(uint8_t slot, uint8_t ch, String range, bool isQuery);
    exit_code scpi_current(uint8_t slot, uint8_t ch, uint16_t value, bool isQuery);
    exit_code scpi_voltage(uint8_t slot, uint8_t ch, uint16_t value, bool isQuery);
    exit_code scpi_measure(uint8_t slot, uint8_t ch, String type);
    exit_code cmd_decomp(String cmd, String* header, String* value, uint8_t* slot, uint8_t* ch, bool* isQuery);
    exit_code scpi_output_switch(uint8_t slot, uint8_t ch, uint8_t refr, bool isQuery);
    exit_code scpi_calibrate(uint8_t slot, uint8_t ch, String value, bool isQuery);
    exit_code scpi_calibrate_reset(void);

    Controller* _controller;
    Backplane* _backplane;
    Daughterboard* _db;
    SMU* _smu;
    uint8_t* _occupancy;
};