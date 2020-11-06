/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef GLOBALS_h
#define GLOBALS_h

#include <Arduino.h>
#include <Console.h>
#include <SPI.h>

#define SERIAL_DEBUG false
#define CONSOLE_DEBUG true
#define SCPI_DEBUG false

// PINOUT
#define BSENABLE_N      11
#define CS0             2
#define CS1             3
#define CS2             4
#define CS3             10
#define DADCS           9
#define FAULTOUT        A1
#define MONITORCS_N     12
#define MONITORRST_N    13
#define POWEROUT        A0
#define SELECT0         5
#define SELECT1         6
#define SELECT2         7
#define SELECT3         8

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

#define MAX_REG_SIZE 32

#define SOCKET_BUFFER_SIZE 1024
#define GENMASK(h, l) (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (31 - (h))))
#define BIT(x)       (1UL << (x)) // Example BIT(2) == 0b100
#define BIT0_FILT(x)         (uint8_t) (((x) & BIT(0)) >> 0)
#define BIT1_FILT(x)         (uint8_t) (((x) & BIT(1)) >> 1)
#define BIT2_FILT(x)         (uint8_t) (((x) & BIT(2)) >> 2)
#define BIT3_FILT(x)         (uint8_t) (((x) & BIT(3)) >> 3)

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
bool merge_spi_devices(spi_device dev1, spi_device dev2, spi_device* dev_out);
void ss_preselect(spi_device dev);
void ss_fast_select(spi_device dev);
void ss_select(spi_device dev);     // preselect followed by fast_select
void ss_deselect(spi_device dev);   // undoes fast_select

#endif
