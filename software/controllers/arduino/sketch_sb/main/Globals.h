/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef GLOBALS_h
#define GLOBALS_h

#include <Arduino.h>
#include <Console.h>
#include <SPI.h>

#define SERIAL_DEBUG  true
#define CONSOLE_DEBUG false
#define SCPI_DEBUG    false

#define MAX_CODE_V     0x8000
#define MAX_CODE_I_CAL 0x2000
#define MAX_CODE_I     0x4000

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

#define GENMASK(h, l) (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (31 - (h))))
#define BIT(x)       (1UL << (x)) // Example BIT(2) == 0b100
#define BIT0_FILT(x)         (uint8_t) (((x) & BIT(0)) >> 0)
#define BIT1_FILT(x)         (uint8_t) (((x) & BIT(1)) >> 1)
#define BIT2_FILT(x)         (uint8_t) (((x) & BIT(2)) >> 2)
#define BIT3_FILT(x)         (uint8_t) (((x) & BIT(3)) >> 3)

#define MAX_CS_PIN_NUMBER  14

typedef struct spi_device {
    uint8_t     cs_pin_num;         // number of SS pins
    uint8_t     cs_pin_list[MAX_CS_PIN_NUMBER];        // pointer to list of SS pins
    uint8_t     cs_pin_states[MAX_CS_PIN_NUMBER];      // pointer to list of SS states (1=HIGH, 0=LOW, usually low)
    uint8_t     cs_fast_pin;        // SS fast enable pin.
                                    // it switches on and off around SPI transfers.
    uint8_t     cs_fast_pin_state;  // SS fast enable pin (state when enabled) (1=HIGH, 0=LOW)
} spi_device;

bool compare_data(uint8_t n_bytes, uint8_t* data, uint8_t* _data);

void copy(uint8_t* src, uint8_t* dst, uint16_t len);

exit_code comm_setup();

bool merge_spi_devices(spi_device dev1, spi_device dev2, spi_device* dev_out);

void ss_preselect(spi_device dev);
void ss_fast_select(spi_device dev);
void ss_select(spi_device dev);     // preselect followed by fast_select
void ss_deselect(spi_device dev);   // undoes fast_select

#endif
