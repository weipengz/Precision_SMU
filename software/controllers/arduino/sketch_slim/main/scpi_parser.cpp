/*
scpi_parser.cpp - scpi parser
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "scpi_parser.h"
#include "backplane.h"


exit_code Parser::parsing(String cmd)
{
    exit_code ret = SUCCESS;
    uint8_t dev;
    uint8_t ch;
    uint8_t slot;
    String param;
    bool isRead;
    String value;
    cmd_decomp(cmd, &dev, &slot, &ch, &param, &isRead, &value);
    #if CONSOLE_DEBUG and SCPI_DEBUG
        Console.println("DEV "+String(dev,DEC)+", Slot "+String(slot,DEC)+", Ch "+String(ch,DEC));
    #endif

    uint8_t idx, hw_addr, reg_addr, len, type, reg_value8;
    uint8_t reg_value[MAX_REG_SIZE];
    uint16_t reg_value16;
    uint32_t reg_mask, reg_value32;
    switch (dev)
    {
    case ADC_DEV: // dev = 0 reg_addr,len,value 
        idx = param.indexOf(',');
        reg_addr = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        idx = param.indexOf(',');
        len = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        #if CONSOLE_DEBUG and SCPI_DEBUG
            Console.println("Reg_addr 0x"+String(reg_addr,HEX)+", Len "+String(len,DEC));
        #endif
        if (isRead){
            max11254_read(getADCDev(slot,ch), reg_addr, len, reg_value);
            for (uint8_t i = 0; i < len; i++) {
                #if SERIAL_DEBUG
                    Serial.println(reg_value[i]);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value[i]);
                #endif
            }
        } 
        else {
            if (len==0){
                #if CONSOLE_DEBUG
                    Console.print("Write CMD: ");
                #endif
                idx = value.indexOf(',');
                reg_value8 = value.substring(0,idx).toInt();
                value.remove(0,idx+1);
                #if CONSOLE_DEBUG
                    Console.println(String(reg_value8,DEC));
                #endif
                max11254_command(getADCDev(slot,ch),reg_value8);
            }
            else {
                #if CONSOLE_DEBUG
                    Console.print("Write: ");
                #endif
                for (uint8_t i = 0; i < len; i++)
                {
                    idx = value.indexOf(',');
                    reg_value[i] = value.substring(0,idx).toInt();
                    value.remove(0,idx+1);
                    #if CONSOLE_DEBUG
                        Console.print(String(reg_value[i],DEC)+"-");
                    #endif
                }
                #if CONSOLE_DEBUG
                    Console.println("");
                #endif
                max11254_write(getADCDev(slot,ch), reg_addr, len, reg_value);
            }
        }
        break;
    case DAC_DEV: // dev = 1 reg_addr, type, value
        idx = param.indexOf(',');
        reg_addr = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        idx = param.indexOf(',');
        type = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        #if CONSOLE_DEBUG and SCPI_DEBUG
            Console.println("Reg_addr 0x"+String(reg_addr,HEX));
        #endif
        if (isRead){
            if (type == 0) {
                ad5753_read(getDACDev(slot,ch), reg_addr, &reg_value16);
                #if SERIAL_DEBUG
                    Serial.println(reg_value16);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value16);
                #endif
            }
            else if (type == 1) {
                ad5753_read32(getDACDev(slot,ch), reg_addr, &reg_value32);
                #if SERIAL_DEBUG
                    Serial.println(reg_value32);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value32);
                #endif
            }
        }
        else {
            if (type == 0) {
                reg_value16 = value.toInt();
                ad5753_write(getDACDev(slot,ch), reg_addr, reg_value16);
                #if CONSOLE_DEBUG
                    Console.print("Write: "+String(reg_value16,DEC));
                #endif
            }
            else if (type == 1) {
                idx = value.indexOf(',');
                reg_mask = value.substring(0,idx).toInt();
                value.remove(0,idx+1);
                reg_value16 = value.toInt();
                ad5753_write_mask(getDACDev(slot,ch), reg_addr, reg_mask, reg_value16);
                #if CONSOLE_DEBUG
                    Console.print("Write Mask: "+String(reg_mask,DEC)+", "+String(reg_value16,DEC));
                #endif
            }
        }
        break;
    case EEPROM_DEV: // dev = 2 reg_addr,len,value
        idx = param.indexOf(',');
        reg_addr = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        idx = param.indexOf(',');
        len = param.substring(0,idx).toInt();
        param.remove(0,idx+1);
        #if CONSOLE_DEBUG and SCPI_DEBUG
            Console.println("Reg_addr 0x"+String(reg_addr,HEX)+", Len "+String(len,DEC));
        #endif
        if (isRead){
            mc25aa02_read(getEEpromDev(slot), reg_addr, len, reg_value);
            for (uint8_t i = 0; i < len; i++) {
                #if SERIAL_DEBUG
                    Serial.println(reg_value[i]);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value[i]);
                #endif
            }
        }
        else {
            #if CONSOLE_DEBUG
                Console.print("Write: ");
            #endif
            for (uint8_t i = 0; i < len; i++)
            {
                idx = value.indexOf(',');
                reg_value[i] = value.substring(0,idx).toInt();
                value.remove(0,idx+1);
                #if CONSOLE_DEBUG
                    Console.print(String(reg_value[i],DEC)+"-");
                #endif
            }
            #if CONSOLE_DEBUG
                Console.println("");
            #endif
            mc25aa02_write(getEEpromDev(slot), reg_addr, len, reg_value);
        }
        break;
    case BPMON_DEV: // dev=3 hw_addr,reg_addr,value
        if(slot == 99) {
            bpMonRst();
            #if CONSOLE_DEBUG
                Console.println("BP MON RST");
            #endif
        } 
        else {
            idx = param.indexOf(',');
            hw_addr = param.substring(0,idx).toInt();
            param.remove(0,idx+1);
            idx = param.indexOf(',');
            reg_addr = param.substring(0,idx).toInt();
            param.remove(0,idx+1);
            #if CONSOLE_DEBUG and SCPI_DEBUG
                Console.println("HW_addr 0x"+String(hw_addr,HEX)+"Reg_addr 0x"+String(reg_addr,HEX));
            #endif
            if (isRead){
                mcp23s17_read(getBpMonDev(), hw_addr, reg_addr, &reg_value8);
                #if SERIAL_DEBUG
                    Serial.println(reg_value8);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value8);
                #endif
            }
            else {
                reg_value8 = value.toInt();
                mcp23s17_write(getBpMonDev(), hw_addr, reg_addr, reg_value8);
            }
        }
        break;
    case DBMON_DEV: // dev=4 hw_addr,reg_addr,value
        if(ch == 99) {
            dbMonRst(slot);
            #if CONSOLE_DEBUG
                Console.println("DB (" + String(slot,DEC) + ") MON RST");
            #endif
        } 
        else {
            idx = param.indexOf(',');
            hw_addr = param.substring(0,idx).toInt();
            param.remove(0,idx+1);
            idx = param.indexOf(',');
            reg_addr = param.substring(0,idx).toInt();
            param.remove(0,idx+1);
            #if CONSOLE_DEBUG and SCPI_DEBUG
                Console.println("HW_addr 0x"+String(hw_addr,HEX)+"Reg_addr 0x"+String(reg_addr,HEX));
            #endif
            if (isRead){
                mcp23s17_read(getDbMonDev(slot), hw_addr, reg_addr, &reg_value8);
                #if SERIAL_DEBUG
                    Serial.println(reg_value8);
                #endif
                #if CONSOLE_DEBUG
                    Console.println(reg_value8);
                #endif
            } 
            else {
                reg_value8 = value.toInt();
                mcp23s17_write(getDbMonDev(slot), hw_addr, reg_addr, reg_value8);
            }
        }
        break;
    case YUN_DEV: // dev=5 
        #if SERIAL_DEBUG
            Serial.println("OK!");
        #endif
        #if CONSOLE_DEBUG
            Console.println("OK!");
        #endif
        break;
    default:
        ret |= SCPI_FAULT;
        break;
    }
    return ret;
}

exit_code Parser::cmd_decomp(String cmd, uint8_t* dev, uint8_t* slot, uint8_t* ch, String* param, bool* isRead, String* value)
{
    exit_code ret = SUCCESS;

    // dev:slot:ch:param? (read)
    // dev:slot:ch:param:value
 
    //cmd dev
    int8_t idx = cmd.indexOf(':');
    if (idx>0){
        *dev = cmd.substring(0,idx).toInt();
    }
    else{
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.remove(0,idx+1);
    //cmd slot
    idx = cmd.indexOf(':');
    if (idx>0){
        *slot = cmd.substring(0,idx).toInt();
    }
    else{
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.remove(0,idx+1);
    //cmd ch
    idx = cmd.indexOf(':');
    if (idx>0){
        *ch = cmd.substring(0,idx).toInt();
    }
    else{
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.remove(0,idx+1);
    //cmd len & value
    idx = cmd.indexOf(':');
    if (idx>0){
        *isRead = false;
        *param = cmd.substring(0,idx);
        cmd.remove(0,idx+1);
        *value = cmd;
    }
    else{
        idx = cmd.indexOf('?');
        if (idx>0){
            *param = cmd.substring(0,idx);
            *isRead = true;
        }
        else{
            ret |= SCPI_FAULT;
            return ret;
        }
    }
    return ret;
}