/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "Globals.h"
#include "Arduino.h"

bool compare_data(uint8_t n_bytes, uint8_t* data, uint8_t* _data)
{
    for (uint8_t idx = 0; idx < n_bytes; idx++)
        if (data[idx]!=_data[idx]) return false;
    return true;
}

void copy(uint8_t* src, uint8_t* dst, uint16_t len) {
    memcpy(dst, src, sizeof(src[0])*len);
}


exit_code comm_setup()
{
    #if SERIAL_DEBUG
        Serial.begin(500000);
        while (!Serial) { ; } // wait for serial port to connect.
    #endif
    #if CONSOLE_DEBUG
        Bridge.begin();
        Console.begin();
    #endif

    return SUCCESS;
}


void ss_preselect(spi_device dev)
{
    for (uint8_t i=0; i<dev.cs_pin_num; i++)
    {
        digitalWrite(dev.cs_pin_list[i], dev.cs_pin_states[i]);
    }
}

void ss_fast_select(spi_device dev)
{
    digitalWrite(dev.cs_fast_pin, dev.cs_fast_pin_state);
}

void ss_select(spi_device dev)
{
    ss_preselect(dev);
    ss_fast_select(dev);
}

void ss_deselect(spi_device dev)
{
    switch(dev.cs_fast_pin_state)
    {
        case LOW:
            digitalWrite(dev.cs_fast_pin, HIGH);
            break;
        case HIGH:
            digitalWrite(dev.cs_fast_pin, LOW);
            break;
        default:
            digitalWrite(dev.cs_fast_pin, HIGH);
    }
}
