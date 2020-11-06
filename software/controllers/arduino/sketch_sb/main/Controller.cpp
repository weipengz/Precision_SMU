/*
Controller.h - Backplane controller functionality
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "Controller.h"

exit_code controller_setup()
{
    pinMode(BS,             OUTPUT);
    pinMode(CS0,            OUTPUT);
    pinMode(CS1,            OUTPUT);
    pinMode(CS2,            OUTPUT);
    pinMode(CS3,            OUTPUT);
    pinMode(DADCS,          OUTPUT);
    pinMode(FAULTOUT,       INPUT_PULLUP);
    pinMode(POWEROUT,       INPUT_PULLUP);
    pinMode(LDAC,           OUTPUT);

    digitalWrite(BS,            LOW);
    digitalWrite(CS0,           LOW);
    digitalWrite(CS1,           LOW);
    digitalWrite(CS2,           LOW);
    digitalWrite(CS3,           HIGH);
    digitalWrite(DADCS,         LOW);
    digitalWrite(LDAC,          LOW);

    SPI.begin();

    // #if SERIAL_DEBUG
    //     Serial.println("CON LOAD");
    // #endif

    return SUCCESS;
}