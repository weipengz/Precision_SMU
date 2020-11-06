/*
main.cxx - main
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "globals.h"
#include "backplane.h"
#include "controller.h"
#include "eeprom.h"
#include "monitor.h"
#include "dac.h"
#include "adc.h"
#include "parser.h"

uint8_t occupancy[16];
Parser parser = Parser(occupancy);

void loop()
{
    char cmd[100];
    scanf("%s",cmd);
    parser.parsing(cmd);
    //~ printf("%s\n",str);
    delay(100);
}

static void shut_down()
{
    printf("Shutting down...\n");
    bcm2835_close(); 
}

int main()
{
    bcm2835_init();
    atexit(shut_down);
    
    imebuild_setup(occupancy);
    
    while(1)
    {
        loop();
    }
    
    return 0;
}
