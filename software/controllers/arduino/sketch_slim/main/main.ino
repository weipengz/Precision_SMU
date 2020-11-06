// Main file

#include "Globals.h"
#include "ControllerLib.h"
#include "Backplane.h"
#include "scpi_parser.h"

Parser parser = Parser();
String inString = "";

void setup() {
    exit_code excode = SUCCESS;
    ctrlSetup();
    #if SERIAL_DEBUG
        serialSetup();
    #endif
    Bridge.begin();
    Console.begin();

    uint8_t estatus;
    for (uint8_t slot = 0; slot < 16; slot++)
    {
        mc25aa02_read_status(getEEpromDev(slot), &estatus);
        if (estatus&0x0C) mc25aa02_write_status(getEEpromDev(slot), 0x00);
    }

    #if SERIAL_DEBUG
        Serial.println("Ready");
    #endif
    #if CONSOLE_DEBUG
        Console.println("Ready");
    #endif
}

void loop() {
    if (Console.available() > 0) {
        inString = Console.readStringUntil('\n');
        // Serial.println(inString);
        // Console.println(inString);
        parser.parsing(inString);
    }
    delay(100);
}