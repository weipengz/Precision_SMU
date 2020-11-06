/*
Globals.h - Global configuration variables
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#ifndef GLOBALS_h
#define GLOBALS_h

#include <Arduino.h>

typedef uint16_t exit_code;

#define SUCCESS         0         // This can be compared to 0, so it has to be <= 0
#define FAILURE         0b1 << 0
#define CAL_FAILURE     0b1 << 1
#define TIMEOUT         0b1 << 2
#define SPI_FAILURE     0b1 << 3
#define AD5753_FAULT    0b1 << 4
#define DAC_RANGE_FAULT 0b1 << 5
#define DAC_MODE_FAULT  0b1 << 6
#define SCPI_FAULT      0b1 << 7
#define MEAS_TYPE_FAULT 0b1 << 8
#define SW_TYPE_FAULT   0b1 << 9
#define SMU_UNAVAILABLE 0b1 << 10

// Enable debug to serial
#define SERIAL_DEBUG true
// Output SPI packets to DEBUG
#define SPI_DEBUG false
// Output SPI selects to DEBUG
#define SPI_PIN_DEBUG false
// Output Register readings to DEBUG
#define REG_DEBUG false
// Output ADC readings to DEBUG
#define ADC_DEBUG false
// Output more verbose AD5753 messages
#define AD5753_DEBUG false
// Output slots and channels occupancy when booting up
#define OCCUPANCY_DEBUG true

// Compile calibration relevant code
#define CAL_COMPILE true
// Compile diagonse relevant code
#define DIAG_COMPILE false
// Compile diagonse verbose relevant code
#define DIAG_verbose false
// Compile AD5753 ADC measurement relevant code
#define ADADC_COMPILE false
// Compile toggle switch relevant code
#define SW_COMPILE false
// Compile EEPROM debug relevant code
#define EEPROM_COMPILE false

#define SOCKET_BUFFER_SIZE 1024

#if SERIAL_DEBUG
#define EARLY_RETURN(x)    \
({\
    if ((x) > 0) { \
        Serial.print("* EARLY RETURN CODE "); \
        Serial.println((x), HEX); \
        return (x); \
    } \
});
#else
#define EARLY_RETURN(x)    if ((x) > 0) return (x);
#endif
/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h.
 * Example: GENMASK(4, 2): = 0b11100
 */
#define GENMASK(h, l) \
        (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (31 - (h))))
/*
 * Create a bitmask where only the i-th bit from the right is  1
 */
#define BIT(x)       (1UL << (x)) // Example BIT(2) == 0b100


#define MAX_CS_PIN_NUMBER       14

typedef struct spi_device {
    uint8_t     cs_pin_num;         // number of SS pins
    uint8_t     cs_pin_list[MAX_CS_PIN_NUMBER];        // pointer to list of SS pins
    uint8_t     cs_pin_states[MAX_CS_PIN_NUMBER];      // pointer to list of SS states (1=HIGH, 0=LOW, usually low)
    uint8_t     cs_fast_pin;        // SS fast enable pin.
                                    // it switches on and off around SPI transfers.
    uint8_t     cs_fast_pin_state;  // SS fast enable pin (state when enabled) (1=HIGH, 0=LOW)
} spi_device;

// Function to copy 'len' elements from 'src' to 'dst'
void copy(uint8_t* src, uint8_t* dst, uint16_t len);

void general_setup();

bool merge_spi_devices(spi_device dev1, spi_device dev2, spi_device* dev_out);

void ss_preselect(spi_device dev);
void ss_fast_select(spi_device dev);
void ss_select(spi_device dev);     // preselect followed by fast_select
void ss_deselect(spi_device dev);   // undoes fast_select

#endif
