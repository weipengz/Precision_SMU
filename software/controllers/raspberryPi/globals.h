/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#ifndef GLOBALS_h
#define GLOBALS_h

#include <iostream>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <bcm2835.h>


//Compile control
#define STD_DEBUG true
#define DAC_DEBUG false
#define REG_DEBUG false
#define SPI_DEBUG false
#define SCPI_DEBUG false

//Error control
typedef uint16_t exit_code;

#define SUCCESS         0 
#define FAILURE         0b1 << 0
#define CAL_FAILURE     0b1 << 1
#define TIMEOUT         0b1 << 2
#define SPI_FAILURE     0b1 << 3
#define DAC_FAULT       0b1 << 4
#define DAC_RANGE_FAULT 0b1 << 5
#define DAC_MODE_FAULT  0b1 << 6
#define SCPI_FAULT      0b1 << 7
#define MEAS_TYPE_FAULT 0b1 << 8
#define SW_TYPE_FAULT   0b1 << 9
#define SMU_UNAVAILABLE 0b1 << 10
#define OVER_RANGE      0b1 << 11


//IO definition
#define CS0 15 //GPIO27 PHY13
#define CS1 16 //GPIO22 PHY15
#define CS2 1 //GPIO5 PHY29
#define CS3 4 //GPIO6 PHY31

#define SELECT0 8 //GPIO2  PHY3
#define SELECT1 9 //GPIO3  PHY5
#define SELECT2 7 //GPIO4  PHY7
#define SELECT3 0 //GPIO17 PHY11

#define DADCS   3 //GPIO16 PHY36
#define MONCS_N 6 //GPIO12 PHY32

#define BSENABLE_N 24 //GPIO19 PHY35
#define MONRST_N   25 //GPIO26 PHY37

#define POWERINT 28 //GPIO21 PHY40
#define FAULTINT 29 //GPIO20 PHY38

#define SW_INTERNAL "INTERNAL"
#define SW_EXTERNAL "EXTERNAL"

#define DEFAULT_RANGE DAC_CONFIG_RANGE_5V_UNI

//Bit wise operations
#define BIT(x)       (1UL<<(x))
#define BIT0_FILT(x) (uint8_t) (((x)&BIT(0))>>0)
#define BIT1_FILT(x) (uint8_t) (((x)&BIT(1))>>1)
#define BIT2_FILT(x) (uint8_t) (((x)&BIT(2))>>2)
#define BIT3_FILT(x) (uint8_t) (((x)&BIT(3))>>3)

#define GENMASK(h,l) (((~0UL)-(1UL<<(l))+1)&(~0UL>>(31-(h))))

//Chip selection functions
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

void ss_preselect(spi_device dev);
void ss_fast_select(spi_device dev);
void ss_select(spi_device dev);     // preselect followed by fast_select
void ss_deselect(spi_device dev);   // undoes fast_select
#endif
