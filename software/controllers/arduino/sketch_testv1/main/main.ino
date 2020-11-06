// Main file

#include <EEPROM.h>
#include <SPI.h>
#include "Globals.h"
#include "NetworkLib.h"
#include "ControllerLib.h"
#include "BackplaneRevA.h"
#include "scpi_parser.h"

// Network network;
Controller controller;
Backplane backplane;
uint8_t occupancy[16]; //an array storing existed slots and channels
Parser parser = Parser(&controller, &backplane, occupancy);
String inString = ""; 

void setup() {
    exit_code excode = SUCCESS;
    for (uint8_t slot = 0; slot < 16; slot++) occupancy[slot] = 0x00;
    
    // put your setup code here, to run once:
    general_setup(); //Init Setial port
    // Setup sequence
    // network.setup();
    controller.setup();

    excode |= backplane.scan_all_slots(occupancy);

    uint16_t opy_db = 0x0000;
    for (uint8_t slot = 0; slot < 16; slot++)
        if (occupancy[slot]>0) opy_db |= BIT(slot);
    excode |= backplane.setup(opy_db);

    Daughterboard db = Daughterboard(0);
    uint8_t data[6] = {1,2,3,4,5,6};
    db.write_eeprom(0,6,data);
    db.read_eeprom(0,6,data);
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(String(data[i])+"-");
    }
    Serial.println();

    #if SERIAL_DEBUG
        if (excode == SUCCESS)
        {
            Serial.println("---> Setup finished successfully.");
        }
        else
        {
            Serial.print("---> Setup finished with error: ");
            Serial.println(excode, HEX);
        }
    #endif
}

void loop() {
    // put your main code here, to run repeatedly:

//    backplane.print_all_mon_regs();
//    delay(10000);

     // db1.print_all_mon_regs();
     // db1.print_mac_address();
     // db1.print_all_slots();
    // smu0.startup_sequence();
    //  delay(10000);

     // db1.flicker_eeprom_spi_dev();
     // delay(1000);

    // network.test_loop();
    // controller.dump_eeprom_to_serial();
    // controller.test_ad5758_serial();

    // controller.setup();

    //  Loop over all input dac codes and measure the output of the
    // current sensor. Measure ten times for each dac code to estimate variance.
    // Origin of variance unknown.

    #if SERIAL_DEBUG
        if( digitalRead(FAULTOUT) + digitalRead(POWEROUT) )
        {
            // Serial.println(digitalRead(FAULTOUT));
            // Serial.println(digitalRead(POWEROUT));
            //do sth checking ad5753 fault
            Daughterboard* db;
            SMU* smu;
            uint16_t dac_fault_slots, power_fault_slots;
            uint8_t dac_fault_chs, power_fault_chs;
            Serial.println("Fault....");
            #if DIAG_COMPILE
            backplane.monitor_diag(&dac_fault_slots, &power_fault_slots);
            // Serial.println(dac_fault_slots,BIN);
            // Serial.println(power_fault_slots,BIN);
            for (uint8_t slot = 0; slot < 16; slot++){
            if( (dac_fault_slots&BIT(slot)) or (power_fault_slots&BIT(slot)) )
            {
                db = new Daughterboard(slot);
                db->monitor_diag(&dac_fault_chs, &power_fault_chs);
                // Serial.println(dac_fault_chs,BIN);
                // Serial.println(power_fault_chs,BIN);
                for (uint8_t ch = 0; ch < 16; ch++){
                    if(dac_fault_chs&BIT(ch))
                    {
                        smu = new SMU(db, ch);
                        Serial.println("ADC FAULT: Slot "+String(slot,DEC)+", Ch "+String(ch,DEC));
                        smu->diagonse();
                        delete(smu);
                    }
                    if(power_fault_chs&BIT(ch))
                    {
                        Serial.println("POWER FAULT: Slot "+String(slot,DEC)+", Ch "+String(ch,DEC));
                    } 
                }
                delete(db);
            } 
            }
            #endif
            delay(3000);
        }
        else
        {
            if(Serial.available() > 0){
            inString = Serial.readStringUntil('\n');
            inString.remove(inString.length()-1); //remove "\n" in the end
            // Serial.println("Receieved: " + inString);
            parser.parsing(inString);
            }
        }
    #endif
    delay(100);
}
