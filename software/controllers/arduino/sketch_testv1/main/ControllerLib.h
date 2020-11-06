/*
Controller.h - Backplane controller functionality
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef Controller_h
#define Controller_h

#include <EEPROM.h>
#include <SPI.h>
#include "Globals.h"
#include "AD5753.h"
#include "MAX11254.h"
#include "BackplaneRevA.h"

// Controller interface description
class Controller
{
public:
    void setup(void);
    // void test_imev2db_serial(void);

    // void dump_eeprom_to_serial(void);

private:
};

#endif
