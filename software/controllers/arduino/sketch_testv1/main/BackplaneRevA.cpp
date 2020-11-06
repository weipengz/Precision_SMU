/*
BackplaneRevA.h - Firmware for Backplane REV A
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "BackplaneRevA.h"


exit_code Backplane::setup(uint16_t opy_db)
{
    exit_code ret = SUCCESS;
    /* Backplane monitors */
    uint8_t dataMSB = uint8_t((opy_db>>8)&0xFF);
    uint8_t dataLSB = uint8_t(opy_db&0xFF);

    // Reset monitor chips
    ret |= monitor_reset();
    // Enable hardware select
    ret |= mcp23s17_iocon_cfg(get_mon_spi_dev(), 0x00, 0xE8);
    ret |= mcp23s17_iocon_cfg(get_mon_spi_dev(), 0x01, 0xE8);
    // Fault Monitor setup
    ret |= mcp23s17_setup(get_mon_spi_dev(), FAULTMON_ADDR, dataLSB, dataMSB);
    // Power-good (PGOOD) Monitor setup
    ret |= mcp23s17_setup(get_mon_spi_dev(), PGOODMON_ADDR, dataLSB, dataMSB);

    return ret;
}

exit_code Backplane::monitor_reset()
{
    exit_code ret = SUCCESS;

    // Hardware reset the monitor chips.
    digitalWrite(MONITORRST_N,  LOW);
    delayMicroseconds(1);
    digitalWrite(MONITORRST_N,  HIGH);
    delayMicroseconds(2);

    #if SERIAL_DEBUG
        Serial.println("----   Monitor Reset   ----");
    #endif

    return ret;
}

exit_code Backplane::monitor_diag(uint16_t* dac_fault, uint16_t* power_fault)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_mon_spi_dev();
    uint8_t dataMSB, dataLSB;
    ret |= mcp23s17_reg_read(dev, 0x01, MCP23S17_REG_INTFA, &dataLSB);
    ret |= mcp23s17_reg_read(dev, 0x01, MCP23S17_REG_INTFB, &dataMSB);
    *dac_fault = (uint16_t(dataMSB)<<8) + dataLSB;
    ret |= mcp23s17_reg_read(dev, 0x00, MCP23S17_REG_INTFA, &dataLSB);
    ret |= mcp23s17_reg_read(dev, 0x00, MCP23S17_REG_INTFB, &dataMSB);
    *power_fault = (uint16_t(dataMSB)<<8) + dataLSB;
    return ret;
}

exit_code Backplane::scan_all_slots(uint8_t* occupancy)
{
    /*Check slot occupancy and channel availability,
    and initialize all the daughterboards and channels*/
    exit_code ret = SUCCESS;
    bool dbIsExist;
    bool chIsExist;
    uint8_t dbNum = 0;
    uint8_t smuNum = 0;
    Daughterboard* db;
    SMU* smu;
    uint8_t calInfo[2];
    bool calFlag;
    for (uint8_t slot = 0; slot < 16; slot++)
    {    
        db = new Daughterboard(slot);
        db->isExist(&dbIsExist);
        if (dbIsExist)
        {
            dbNum++;
            #if SERIAL_DEBUG and OCCUPANCY_DEBUG
                Serial.print("Slot "+String(slot,DEC)+": ");
            #endif
            calFlag = false;
            for (uint8_t ch = 0; ch < 8; ch++)
            {
                smu = new SMU(db, ch);
                smu->isExist(&chIsExist);
                // chIsExist = true;
                // if(ch==7) chIsExist = true;
                if (chIsExist) 
                {
                    smuNum++;
                    occupancy[slot] |= BIT(ch);
                    ret |= smu->startup_sequence();
                    ret |= smu->select_voltage_mode(SMU_DEFAULT_RANGE);
                    ret |= smu->select_range(true, 1);    
                    #if SERIAL_DEBUG and OCCUPANCY_DEBUG
                        Serial.print("Ch "+String(ch,DEC)+": Good");
                        db->read_eeprom((ch<<4),2,calInfo);
                        if ((calInfo[0]==0x18)&&(calInfo[1]==0x43)) 
                        {
                            calFlag = true;
                            Serial.print(" Y");
                        }
                        else Serial.print(" N");
                    #endif
                    Serial.print(", ");
                }
                else
                {
                    #if SERIAL_DEBUG and OCCUPANCY_DEBUG
                        Serial.print("Ch "+String(ch,DEC)+": Bad, ");
                    #endif
                }
                delete(smu);
            }
            Serial.println("");
            #if SERIAL_DEBUG and OCCUPANCY_DEBUG
                // if (calFlag) Serial.println("Calibration loaded");
                // else Serial.println("No calibration found");
            #endif
            ret |= db->setup(occupancy[slot]);
        }
        else
        {
            #if SERIAL_DEBUG and OCCUPANCY_DEBUG
                Serial.println("Slot "+String(slot,DEC)+" is not used.");
            #endif
        }
        delete(db);
    }
    #if SERIAL_DEBUG
        Serial.println("Found "+String(dbNum,DEC)+" daughterboard(s), and "+String(smuNum,DEC)+" Channel(s) in total.");
    #endif
    return ret;
}

#if EEPROM_COMPILE and SERIAL_DEBUG
exit_code Backplane::print_all_mon_regs()
{
    exit_code ret = SUCCESS;
    uint8_t all_regs[22];

    ret |= mcp23s17_read_all_regs(get_mon_spi_dev(), FAULTMON_ADDR, all_regs);
    EARLY_RETURN(ret)

    Serial.println("FAULTMON registers: ");
    for(uint8_t i = 0; i<22; i++){
        Serial.print(' ');
        Serial.print(i, HEX);
        Serial.print('-');
        Serial.println(all_regs[i], HEX);
    }
    Serial.println();

    ret |= mcp23s17_read_all_regs(get_mon_spi_dev(), PGOODMON_ADDR, all_regs);
    EARLY_RETURN(ret)

    Serial.println("PGOODMON registers: ");
    for(uint8_t i = 0; i<22; i++){
        Serial.print(' ');
        Serial.print(i, HEX);
        Serial.print('-');
        Serial.println(all_regs[i], HEX);
    }
    Serial.println();

    return ret;
}
#endif

spi_device Backplane::get_mon_spi_dev()
{
    #define CS_NUM 1

    uint8_t cs_pin_list[CS_NUM] = {
        BSENABLE_N
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH//LOW
    };

    uint8_t cs_fast_pin = MONITORCS_N;
    uint8_t cs_fast_pin_state = LOW;

    spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

Daughterboard::Daughterboard(uint8_t index):
    index(index)
{
}

exit_code Daughterboard::setup(uint8_t opy_ch)
{
    exit_code ret = SUCCESS;

    /* Backplane monitors */
    // Reset monitor chips
    ret |= monitor_reset();
    // Enable hardware select
    ret |= mcp23s17_iocon_cfg(get_mon_spi_dev(), 0x00, 0xAA);//0xA8
    // Fault Monitor setup
    ret |= mcp23s17_setup(get_mon_spi_dev(), 0x00, opy_ch, opy_ch);

    return ret;
}

exit_code Daughterboard::isExist(bool* exist)
{
    uint8_t mac[6];
    // read_eeprom_status(&estatus);
    read_eeprom(0xFA, 6, mac);
    *exist = ( (mac[0]==0x80) && (mac[1]==0x1f) && (mac[2]==0x12) );
    // Serial.println(estatus,HEX);
}

#if EEPROM_COMPILE
exit_code Daughterboard::read_eeprom_status(uint8_t* estatus)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_spi_dev();
    ret |= mc24aa02_read_status(dev, estatus);
    return SUCCESS;
}
#endif

exit_code Daughterboard::read_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_spi_dev();
    ret |= mc24aa02_read_mem_addr(dev, addr, n_bytes, data);
    return SUCCESS;
}

exit_code Daughterboard::write_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_spi_dev();
    for (uint8_t i = 0; i < 10; i++)
    ret |= mc24aa02_write_mem_addr(dev, addr, n_bytes, data);
    delayMicroseconds(500);
    uint8_t estatus;
    do
    {
        ret |= mc24aa02_read_status(dev, &estatus);
        delayMicroseconds(500);
    }
    while ( (estatus&0x01)==0x01 );
    
    return SUCCESS;
}

#if EEPROM_COMPILE
#define MAC_LENGTH 6
exit_code Daughterboard::read_mac_address(uint8_t* mac_addr)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_spi_dev();
    ret |= mc24aa02_read_mem_addr(dev, 0xFA, MAC_LENGTH, mac_addr);
    return ret;
}
#endif

exit_code Daughterboard::monitor_reset()
{
    exit_code ret = SUCCESS;
    spi_device dev = get_monitor_reset_spi_dev();

    ss_select(dev);
    ss_deselect(dev);

    #if SERIAL_DEBUG
        Serial.print("---- DB(");
        Serial.print(index);
        Serial.println(") Mon. Reset ----");
    #endif

    return ret;
}

exit_code Daughterboard::monitor_diag(uint8_t* adc_fault, uint8_t* power_fault)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_mon_spi_dev();
    ret |= mcp23s17_reg_read(dev, 0x00, MCP23S17_REG_INTFA, adc_fault);
    ret |= mcp23s17_reg_read(dev, 0x00, MCP23S17_REG_INTFB, power_fault);
    return ret;
}

spi_device Daughterboard::get_monitor_reset_spi_dev()
{
    #define CS_NUM 9

    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        HIGH,
        HIGH,
        LOW,
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
    };

    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;//HIGH

    spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

spi_device Daughterboard::get_mon_spi_dev()
{
    #define CS_NUM 9

    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        LOW,
        LOW,
        HIGH,
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
    };

    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;//HIGH

    struct spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

#if EEPROM_COMPILE and SERIAL_DEBUG
#define EEPROM_MEM_LENGTH       256UL
exit_code Daughterboard::print_all_slots()
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_spi_dev();
    uint8_t all_mem_slots[EEPROM_MEM_LENGTH];
    ret |= mc24aa02_read_mem_addr(dev, 0x00, EEPROM_MEM_LENGTH, all_mem_slots);

    Serial.print("DB(");
    Serial.print(index);
    Serial.print(    "): MEM: ");

    for (uint16_t i = 0; i < EEPROM_MEM_LENGTH; i++)
    {
        Serial.print(i, HEX);
        Serial.print('/');
        Serial.print(all_mem_slots[i], HEX);
        Serial.print('.');
    }
    Serial.println();
    return ret;
}

exit_code Daughterboard::print_mac_address()
{
    uint8_t mac_addr[MAC_LENGTH];
    exit_code ret = SUCCESS;

    ret |= read_mac_address(mac_addr);
    Serial.print("DB(");
    Serial.print(index);
    Serial.print(    "): MAC ADDRESS: ");
    for (uint8_t i = 0; i < MAC_LENGTH; i++)
        {
            Serial.print(mac_addr[i], HEX);
            Serial.print('-');
        }

    Serial.println();
    return ret;
}

exit_code Daughterboard::flicker_eeprom_spi_dev(void)
{
    spi_device dev = get_eeprom_spi_dev();

    Serial.print("DB(");
    Serial.print(index);
    Serial.println(    "):");

    Serial.println("ON");
    ss_select(dev);
    delay(1000);
    Serial.println("OFF");
    ss_deselect(dev);
    return SUCCESS;
}

exit_code Daughterboard::print_all_mon_regs()
{
    exit_code ret = SUCCESS;
    uint8_t all_regs[22];

    ret |= mcp23s17_read_all_regs(get_mon_spi_dev(), 0x00, all_regs);

    Serial.print("DB(");
    Serial.print(index);
    Serial.print(    ") ");
    Serial.println("Monitor registers: ");

    for(uint8_t i = 0; i<22; i++){
        Serial.print(' ');
        Serial.print(i, HEX);
        Serial.print('-');
        Serial.println(all_regs[i], HEX);
    }
    Serial.println();

    return ret;
}
#endif

spi_device Daughterboard::get_eeprom_spi_dev()
{
    #define CS_NUM 9

    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        LOW,
        LOW,
        LOW,
        BIT3_FILT(index),
        BIT2_FILT(index),
        BIT1_FILT(index),
        BIT0_FILT(index)
    };

    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;//HIGH

    spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

/* SMU */
/* Public methods */
SMU::SMU (Daughterboard* db, uint8_t channel):
    _db(db),
    _channel(channel)
{
    _gain_i = 0x800000;
    _offset_i = 0x800000;
    _gain_v = 0x800000;
    _offset_v = 0x800000;
}

SMU::SMU (Daughterboard* db, uint8_t channel, uint32_t* calInfo):
    _db(db),
    _channel(channel)
{
    _gain_i = calInfo[0];
    _offset_i = calInfo[1];
    _gain_v = calInfo[2];
    _offset_v = calInfo[3];
}

exit_code SMU::startup_sequence()
{
    exit_code ret = SUCCESS;

    // DAC Startup
    ret |= ad5753_startup_sequence(get_dac_spi_dev());
    EARLY_RETURN(ret)

    // ADC Startup
    ret |= max11254_setup(get_adc_spi_dev());
    EARLY_RETURN(ret)
    ret |= max11254_calibrate(get_adc_spi_dev(), 0b00);
    EARLY_RETURN(ret)

    // Output switches
    ret |= toggle_reference_switch(true);
    EARLY_RETURN(ret)
    ret |= output_switch(true);
    EARLY_RETURN(ret)

    return ret;
}

exit_code SMU::toggle_reference_switch(bool REFRES)
{
    /* Set REFRES to true to switch outputs (LOAD, VSENSE)
       to a reference resistor (RTEST).
       Default true.
     */
    return ad5753_reg_write_mask(
        get_dac_spi_dev(),
        AD5753_REG_GPIO_DATA,
        AD5753_REG_GPIO_DATA_GPO_1_WRITE_MSK,
        AD5753_REG_GPIO_DATA_GPO_1_WRITE_MODE(REFRES)
        );
}

exit_code SMU::output_switch(bool ENABLE)
{
    /* Set ENABLE to true if you want the output switch to work.
       Otherwise, LOAD+, +VSENSE, -VSENSE will be floating.
       Default false.
     */
    return ad5753_reg_write_mask(
        get_dac_spi_dev(),
        AD5753_REG_GPIO_DATA,
        AD5753_REG_GPIO_DATA_GPO_0_WRITE_MSK,
        AD5753_REG_GPIO_DATA_GPO_0_WRITE_MODE(!ENABLE)
        );
}

#if SW_COMPILE
exit_code SMU::switch_output_target(bool REFRES)
{
    exit_code ret = SUCCESS;
    uint16_t current_dac_code;
    ret |= ad5753_pause_output(get_dac_spi_dev(), &current_dac_code);
    delayMicroseconds(200);
    ret |= output_switch(false);
    delayMicroseconds(200);
    ret |= toggle_reference_switch(REFRES);
    delayMicroseconds(200);
    ret |= output_switch(true);
    delayMicroseconds(200);
    ret |= ad5753_set_dac_code(get_dac_spi_dev(), _range, current_dac_code);
    return ret;
}
#endif

// Source Functions
exit_code SMU::select_current_mode(uint8_t range)
{
    /* Sets the DAC to current mode.
     * Side effects: if it was in voltage mode, it will set the output to 0mA.
     */

    exit_code ret = SUCCESS;

    ret |= ad5753_set_new_range(get_dac_spi_dev(), range);
    EARLY_RETURN(ret)

    _dac_mode =  SMU_CURRENT_MODE;
    _range = range;

    return ret;
}

exit_code SMU::select_voltage_mode(uint8_t range)
{
    /* Sets the DAC to voltage mode.
     * Side effects: if it was in current mode, it will set the output to 0V.
     */
    exit_code ret = SUCCESS;

    ret |= ad5753_set_new_range(get_dac_spi_dev(), range);
    EARLY_RETURN(ret)

    _dac_mode =  SMU_VOLTAGE_MODE;
    _range = range;

    return ret;
}

exit_code SMU::set_current(uint16_t dac_code)
{
    exit_code ret = SUCCESS;
    if (_dac_mode != SMU_CURRENT_MODE)
    {
        return ret | DAC_MODE_FAULT;
    }

    ret |= ad5753_set_dac_code(get_dac_spi_dev(), _range, dac_code);
    return ret;
}

exit_code SMU::set_voltage(uint16_t dac_code)
{
    exit_code ret = SUCCESS;
    if (_dac_mode != SMU_VOLTAGE_MODE)
    {
        return ret | DAC_MODE_FAULT;
    }

    ret |= ad5753_set_dac_code(get_dac_spi_dev(), _range, dac_code);
    return ret;
}

exit_code SMU::select_range(bool pga, uint8_t gain)
{
    if(pga)
    {
        return max11254_prepare_convert(get_adc_spi_dev(), gain);
    }
    else
    {
        return max11254_prepare_convert(get_adc_spi_dev(), 0);
    }
}

exit_code SMU::measure_sense_voltage(uint32_t* adc_code)
{
    exit_code ret = SUCCESS;
    uint32_t adc_code_raw;
    ret |= max11254_convert(get_adc_spi_dev(), 2, 0b0111, &adc_code_raw);
    // *adc_code = uint32_t(float(_gain_v)/0x800000*(int32_t(adc_code_raw)-0x800000))+_offset_v;
    *adc_code = uint32_t( (float(_gain_v)/0x800000*0.1+0.9)*int32_t(adc_code_raw)+_offset_v-0x800000);
    return ret;
}

exit_code SMU::measure_load_voltage(uint32_t* adc_code)
{
    exit_code ret = SUCCESS;
    uint32_t adc_code_raw;
    ret |= max11254_convert(get_adc_spi_dev(), 1, 0b0111, &adc_code_raw);
    // *adc_code = uint32_t(float(_gain_v)/0x800000*(int32_t(adc_code_raw)-0x800000))+_offset_v;
    *adc_code = uint32_t( (float(_gain_v)/0x800000*0.1+0.9)*int32_t(adc_code_raw)+_offset_v-0x800000);
    return ret;
}

exit_code SMU::measure_source_current(uint32_t* adc_code)
{
    exit_code ret = SUCCESS;
    uint32_t adc_code_raw;
    ret |= max11254_convert(get_adc_spi_dev(), 3, 0b0111, &adc_code_raw);
    // *adc_code = uint32_t(float(_gain_i)/0x800000*(int32_t(adc_code_raw)-0x800000))+_offset_i;
    *adc_code = uint32_t( (float(_gain_i)/0x800000*0.1+0.9)*int32_t(adc_code_raw)+_offset_i-0x800000);
    return ret;
}

// exit_code SMU::adc_hw_reset()
// {
//     exit_code ret = SUCCESS;

//     ret |= ad5753_reg_write_mask(
//         get_dac_spi_dev(),
//         AD5753_REG_GPIO_DATA,
//         AD5753_REG_GPIO_DATA_GPO_2_WRITE_MSK,
//         AD5753_REG_GPIO_DATA_GPO_2_WRITE_MODE(0x0)
//         );
//     // no EARLY_RETURN

//     ret |= ad5753_reg_write_mask(
//         get_dac_spi_dev(),
//         AD5753_REG_GPIO_DATA,
//         AD5753_REG_GPIO_DATA_GPO_2_WRITE_MSK,
//         AD5753_REG_GPIO_DATA_GPO_2_WRITE_MODE(0x1)
//         );
//     EARLY_RETURN(ret)

//     // A hw reset from any state to STANDBY takes 10ms cf. page 29.
//     delay(10);
//     return ret;
// }

#if ADADC_COMPILE
exit_code SMU::read_debug_adc1(uint16_t* adc_data)
{
    return ad5753_read_adc1(get_dac_spi_dev(), adc_data);
}

exit_code SMU::read_debug_adc2(uint16_t* adc_data)
{
    return ad5753_read_adc2(get_dac_spi_dev(), adc_data);
}
#endif

exit_code SMU::read_debug_adc(uint8_t ip_address, uint16_t* adc_data)
{
    return ad5753_adc_single_conversion(get_dac_spi_dev(), ip_address, adc_data);
}

/* Private methods */

spi_device SMU::get_spi_device(bool dadcs)
{
    #define CS_NUM 10

    uint8_t cs_pin_list[CS_NUM] = {
        MONITORCS_N,
        CS3,
        CS2,
        CS1,
        CS0,
        SELECT3,
        SELECT2,
        SELECT1,
        SELECT0,
        DADCS
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        LOW,
        BIT2_FILT(_channel),
        BIT1_FILT(_channel),
        BIT0_FILT(_channel),
        BIT3_FILT(_db->index),
        BIT2_FILT(_db->index),
        BIT1_FILT(_db->index),
        BIT0_FILT(_db->index),
        dadcs
    };

    uint8_t cs_fast_pin = BSENABLE_N;
    uint8_t cs_fast_pin_state = LOW;//HIGH

    spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

spi_device SMU::get_dac_spi_dev()
{
    return get_spi_device(true);
}

spi_device SMU::get_adc_spi_dev()
{
    return get_spi_device(false);
}

exit_code SMU::get_range_mode(uint8_t* range, uint8_t* dac_mode)
{
    exit_code ret = SUCCESS;
    uint16_t dac_config_reg;
    ret |= ad5753_reg_read(get_dac_spi_dev(), AD5753_REG_DAC_CONFIG, &dac_config_reg);
    *range = uint8_t(dac_config_reg & 0xF);
    if (AD5753_DAC_CONFIG_CURRENT_MODE(*range)) *dac_mode = SMU_CURRENT_MODE;
    else *dac_mode = SMU_VOLTAGE_MODE;
    _range = *range;
    _dac_mode = *dac_mode;
    return ret;
}

exit_code SMU::get_dac_code(uint16_t* dac_code)
{
    exit_code ret = SUCCESS;
    ret |= ad5753_reg_read(get_dac_spi_dev(), AD5753_REG_DAC_INPUT, dac_code);
    return ret;
}

#if DIAG_COMPILE
exit_code SMU::diagonse()
{
    exit_code ret = SUCCESS;
    spi_device dev = get_dac_spi_dev();
    uint32_t _status_data;
    uint16_t _di_diag, _an_diag;
    ret |= ad5753_reg_read32(dev, AD5753_REG_STATUS, &_status_data);
    Serial.print("STATUS: ");
    Serial.println(_status_data, HEX);
    ret |= ad5753_reg_read(dev, AD5753_REG_DIGITAL_DIAG_RESULTS, &_di_diag);
    Serial.print("DIGITAL_DIAG_RESULTS: ");
    Serial.println(_di_diag, HEX);
#if DIAG_verbose
    if(_di_diag>0)
    {
        if(_di_diag & AD5753_DI_DIAG_SPI_CRC_ERR)
        {
            Serial.print("SPI CRC, ");
        }
        if(_di_diag & AD5753_DI_DIAG_SLIPBIT_ERR)
        {
            Serial.print("DIAG SLIPBIT, ");
        }
        if(_di_diag & AD5753_DI_DIAG_SCLK_COUNT_ERR)
        {
            Serial.print("SCLK COUNT, ");
        }
        if(_di_diag & AD5753_DI_DIAG_INVALID_SPI_ACC_ERR)
        {
            Serial.print("INVALID SPI ACC, ");
        }
        if(_di_diag & AD5753_DI_DIAG_CAL_MEM_CRC_ERR)
        {
            Serial.print("CAL_MEM_CRC, ");
        }
        if(_di_diag & AD5753_DI_DIAG_INV_DAC_CHECK_ERR)
        {
            Serial.print("INV DAC CHECK, ");
        }
        if(_di_diag & AD5753_DI_DIAG_DAC_LATCH_MON_ERR)
        {
            Serial.print("DAC LATCH MON, ");
        }
        if(_di_diag & AD5753_DI_DIAG_3WI_RC_ERR)
        {
            Serial.print("3WI RC, ");
        }
        if(_di_diag & AD5753_DI_DIAG_WDT_ERR)
        {
            Serial.print("WDT, ");
        }
        if(_di_diag & AD5753_DI_DIAG_ERR_3WI)
        {
            Serial.print("3WI, ");
        }
        if(_di_diag & AD5753_DI_DIAG_RES_OCCURRED)
        {
            Serial.print("RES_OCCURRED, ");
        }
        if(_di_diag & AD5753_DI_DIAG_SLEW_BUSY)
        {
            Serial.print("SLEW BUSY, ");
        }
        if(_di_diag & AD5753_DI_DIAG_CAL_MEM_UNREFRESHED)
        {
            Serial.print("CAL MEM UNREFRESHED, ");
        }
        Serial.println("");
    }
#endif

    ret |= ad5753_reg_read(dev, AD5753_REG_ANALOG_DIAG_RESULTS, &_an_diag);
    Serial.print("ANALOG_DIAG_RESULTS: ");
    Serial.println(_an_diag, HEX);
#if DIAG_verbose
    if(_an_diag>0)
    {
        if(_an_diag & AD5753_AN_DIAG_VLDO_ERR)
        {
            Serial.print("VLDO, ");
        }
        if(_an_diag & AD5753_AN_DIAG_INT_AVCC_ERR)
        {
            Serial.print("INT AVCC, ");
        }
        if(_an_diag & AD5753_AN_DIAG_REFIN_ERR)
        {
            Serial.print("REFIN, ");
        }
        if(_an_diag & AD5753_AN_DIAG_REFOUT_ERR)
        {
            Serial.print("REFOUT, ");
        }
        if(_an_diag & AD5753_AN_DIAG_MAIN_DIE_TEMP_ERR)
        {
            Serial.print("MAIN DIE TEMP, ");
        }
        if(_an_diag & AD5753_AN_DIAG_DCDC_DIE_TEMP_ERR)
        {
            Serial.print("DCDC DIE TEMP, ");
        }
        if(_an_diag & AD5753_AN_DIAG_VOUT_SC_ERR)
        {
            Serial.print("VOUT SC, ");
        }
        if(_an_diag & AD5753_AN_DIAG_IOUT_OC_ERR)
        {
            Serial.print("IOUT OC, ");
        }
        if(_an_diag & AD5753_AN_DIAG_DCDC_P_PWR_ERR)
        {
            Serial.print("DCDC P PWR, ");
        }
        if(_an_diag & AD5753_AN_DIAG_DCDC_P_SC_ERR)
        {
            Serial.print("DCDC P SC, ");
        }
        if(_an_diag & AD5753_AN_DIAG_VIOUT_OV_ERR)
        {
            Serial.print("VIOUT OV, ");
        }
        Serial.println("");
    }
#endif
    return ret;
}
#endif

exit_code SMU::isExist(bool* exist)
{
    exit_code ret = SUCCESS;
    uint16_t chip_id;
    ret |= ad5753_reg_read(get_dac_spi_dev(), AD5753_REG_CHIP_ID, &chip_id);
    *exist = ((chip_id!=0xFFFF) && (chip_id!=0x0000));
    // Serial.print("chip_id:"+String(chip_id,HEX)+" ");
    return ret;
}