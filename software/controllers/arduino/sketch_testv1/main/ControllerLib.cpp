/*
Controller.h - Backplane controller functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include <EEPROM.h>
#include <SPI.h>
#include <Arduino.h>
#include "ControllerLib.h"

void Controller::setup()
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

    digitalWrite(BSENABLE_N,    HIGH);//LOW
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

    SPI.begin();

    #if SERIAL_DEBUG
        Serial.println("---- Controller Loaded ----");
    #endif
}

// #if SERIAL_DEBUG
//     void Controller::dump_eeprom_to_serial()
//     {
//         int address = 0;
//         byte value;
//         while (address < EEPROM.length())
//         {
//             // read a byte from the current address of the EEPROM
//             value = EEPROM.read(address);

//             Serial.print(address);
//             Serial.print("/");
//             Serial.print(EEPROM.length());
//             Serial.print("\t");
//             Serial.print(value, DEC);
//             Serial.println();

//             /***
//                 Advance to the next address, when at the end restart at the beginning.

//                 Larger AVR processors have larger EEPROM sizes, E.g:
//                 - Arduno Duemilanove: 512b EEPROM storage.
//                 - Arduino Uno:        1kb EEPROM storage.
//                 - Arduino Mega:       4kb EEPROM storage.

//                 Rather than hard-coding the length, you should use the pre-provided length function.
//                 This will make your code portable to all AVR processors.
//             ***/
//             address = address + 1;
//             if (address == EEPROM.length()) {
//                 address = 0;
//             }

//             /***
//                 As the EEPROM sizes are powers of two, wrapping (preventing overflow) of an
//                 EEPROM address is also doable by a bitwise and of the length - 1.

//                 ++address &= EEPROM.length() - 1;
//             ***/

//             delay(50);
//         }
//     }
// #endif
