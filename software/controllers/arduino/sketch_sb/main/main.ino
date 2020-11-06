// Main file

#include "Globals.h"
#include "Controller.h"
#include "SMUBoard.h"
#include "Parser.h"

// Network network;
uint8_t occupancy; //an parameters storing existed channels
Parser parser = Parser(&occupancy);
String inString = ""; 

void setup() {
    exit_code excode = SUCCESS;

    occupancy = BIT(2)|BIT(3);

    excode |= comm_setup();
    excode |= controller_setup();
    excode |= board_setup(&occupancy);

    #if SERIAL_DEBUG
        if (excode == SUCCESS)
        {
            Serial.println("SETUP OK");
        }
        else
        {
            Serial.print("SETUP ERR:");
            Serial.println(excode, HEX);
        }
    #endif
    #if CONSOLE_DEBUG
        if (excode == SUCCESS)
        {
            Console.println("SETUP OK");
        }
        else
        {
            Console.print("SETUP ERR:");
            Console.println(excode, HEX);
        }
    #endif
}

void loop() {
    #if SERIAL_DEBUG
        if(Serial.available() > 0)
        {
            inString = Serial.readStringUntil('\n');
            // Serial.println("cmd["+inString+"]");
            parser.parsing(inString);
        }
    #endif
    #if CONSOLE_DEBUG
        if(Console.available() > 0)
        {
            inString = Console.readStringUntil('\r');
            // inString.substring(0,inString.length()-1);
            // Console.println("cmd["+inString+"]");
            parser.parsing(inString);
            Console.readStringUntil('\n');
        }
    #endif
    delay(100);
}
