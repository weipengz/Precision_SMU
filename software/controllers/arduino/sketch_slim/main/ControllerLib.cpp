/*
Controller.h - Backplane controller functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "ControllerLib.h"

void ctrlSetup()
{
    // IMEBUILD V2 DB
    pinMode(BSENABLE_N,     OUTPUT);
    pinMode(CS0,            OUTPUT);
    pinMode(CS1,            OUTPUT);
    pinMode(CS2,            OUTPUT);
    pinMode(CS3,            OUTPUT);
    pinMode(DADCS,          OUTPUT);
    pinMode(FAULTOUT,       INPUT_PULLUP);
    pinMode(MONITORCS_N,    OUTPUT);
    pinMode(MONITORRST_N,   OUTPUT);
    pinMode(POWEROUT,       INPUT_PULLUP);
    pinMode(SELECT0,        OUTPUT);
    pinMode(SELECT1,        OUTPUT);
    pinMode(SELECT2,        OUTPUT);
    pinMode(SELECT3,        OUTPUT);

    digitalWrite(BSENABLE_N,    HIGH);
    digitalWrite(CS0,           LOW);
    digitalWrite(CS1,           LOW);
    digitalWrite(CS2,           LOW);
    digitalWrite(CS3,           HIGH);
    digitalWrite(DADCS,         LOW);
    digitalWrite(MONITORCS_N,   HIGH);
    digitalWrite(MONITORRST_N,  HIGH);
    digitalWrite(SELECT0,       LOW);
    digitalWrite(SELECT1,       LOW);
    digitalWrite(SELECT2,       LOW);
    digitalWrite(SELECT3,       LOW);

    // start SPI
    SPI.begin();
}

void serialSetup()
{
    #if SERIAL_DEBUG
        // initialize serial communication:
        Serial.begin(500000);
        while (!Serial) { ; } // wait for serial port to connect.
    #endif
}