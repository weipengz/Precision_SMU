#include "scpi_parser.h"

Parser::Parser(Controller* controller, Backplane* backplane, uint8_t* occupancy):
    _controller(controller),
    _backplane(backplane),
    _occupancy(occupancy)
{
}

exit_code Parser::parsing(String cmd)
{
    exit_code ret = SUCCESS;
    if (cmd[0]=='*') {//Common Command
        cmd.remove(0,1);
        if (cmd==SCPI_IDN){
            ret |= scpi_idn();
            return ret;
        }
        if (cmd==SCPI_RST){
            ret |= scpi_rst();
            return ret;
        }
#if CAL_COMPILE
        if (cmd==SCPI_CAL){
            ret |= scpi_cal();
            return ret;
        }
        if (cmd==SCPI_CALC){
            ret |= scpi_calibrate_reset();
            return ret;
        }
#endif
        if (cmd==SCPI_OCC){
            ret |= scpi_occ();
            return ret;
        }
    }
    else{ //Funtional Command
        String header;
        String value;
        uint8_t ch;
        uint8_t slot;
        bool isQuery;
        cmd_decomp(cmd, &header, &value, &slot, &ch, &isQuery);
        //Check channel availablity
        if ( !(_occupancy[slot]&BIT(ch)) )
        {
            Serial.println("Channel Error: Invalid slot or channel");
            return ret | SMU_UNAVAILABLE;
        }
        //Execute
        if (header==SCPI_CONF){
            ret |= scpi_config(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CURR){
            ret |= scpi_current(slot, ch, uint16_t(value.toInt()), isQuery);
            return ret;
        }
        if (header==SCPI_VOLT){
            ret |= scpi_voltage(slot, ch, uint16_t(value.toInt()), isQuery);
            return ret;
        }
        if (header==SCPI_MEAS){
            ret |= scpi_measure(slot, ch, value);
            return ret;
        }
#if SW_COMPILE
        if (header==SCPI_REFR){
            ret |= scpi_output_switch(slot, ch, uint8_t(value.toInt()), isQuery);
            return ret;
        }
#endif
#if CAL_COMPILE
        if (header==SCPI_CALS){
            ret |= scpi_calibrate(slot, ch, value, isQuery);
            return ret;
        }
#endif
    }
    Serial.println("Syntax Error: Invalid Command");
    return ret | SCPI_FAULT;
}

exit_code Parser::cmd_decomp(String cmd, String* header, String* value, uint8_t* slot, uint8_t* ch, bool* isQuery)
{
    exit_code ret = SUCCESS;
    //cmd slot
    int8_t idx = cmd.indexOf(':');
    if (idx>0){
        *slot = cmd.substring(0,idx).toInt();
    }
    else{
        Serial.println("Syntax Error: no valid slot");
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.remove(0,idx+1);
    //cmd channel
    idx = cmd.indexOf(':');
    if (idx>0){
        *ch = cmd.substring(0,idx).toInt();
    }
    else{
        Serial.println("Syntax Error: no valid channel");
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.remove(0,idx+1);
    //cmd header
    idx = cmd.indexOf(':');
    if (idx>0){
        *header = cmd.substring(0,idx);
        *isQuery = false;
    }
    else{
        idx = cmd.indexOf('?');
        if (idx>0){
            *header = cmd.substring(0,idx);
            *isQuery = true;
        }
        else{
            Serial.println("Syntax Error: no valid value");
            ret |= SCPI_FAULT;
            return ret;
        }
    }
    //cmd value
    cmd.remove(0,idx+1);
    *value = cmd;

    #if SCPI_DEBUG
        Serial.print("Slot: ");
        Serial.print(*slot, DEC);
        Serial.print(", ");
        Serial.print("Ch: ");
        Serial.print(*ch, DEC);
        Serial.print(", ");
        Serial.print("Header: "+*header+", ");
        Serial.print("Value: ");
        Serial.print(*value);
        Serial.print(", ");
        Serial.print("isQuery: ");
        Serial.print(*isQuery);
        Serial.println();
    #endif

    return ret;
}

exit_code Parser::scpi_idn(){
    exit_code ret = SUCCESS;
    Serial.println("IMEBUILD V2 Rev B");

    Daughterboard* db = new Daughterboard(8);
    uint8_t calMem[14];
    for (uint8_t idx = 0; idx < 14; idx++) calMem[idx] = 0x00;

    db->write_eeprom(0x00,14,calMem);

    return ret;
}

exit_code Parser::scpi_rst()
{
    Serial.println("Resetting...");
    exit_code ret = SUCCESS;
    for (uint8_t slot = 0; slot < 16; slot++) _occupancy[slot] = 0x00;
    general_setup();
    // network.setup();
    _controller->setup();

    ret |= _backplane->scan_all_slots(_occupancy);

    uint16_t opy_db = 0x0000;
    for (uint8_t slot = 0; slot < 16; slot++)
        if (_occupancy[slot]>0) opy_db |= BIT(slot);
    ret |= _backplane->setup(opy_db);

    #if SERIAL_DEBUG
        if (ret == SUCCESS)
        {
            Serial.println("---> Setup finished successfully.");
        }
        else
        {
            Serial.print("---> Setup finished with error: ");
            Serial.println(ret, HEX);
        }
    #endif
    return ret;
}

#if CAL_COMPILE
exit_code Parser::scpi_cal()
{//Calibration function
    exit_code ret = SUCCESS;
    Serial.println("Calibrating...");

    uint16_t value = 0;
    uint32_t i_sense = 0;
    uint32_t v_sense = 0;
    uint32_t v_load = 0;
    uint8_t calMem[14];
    uint32_t calInfo[4];

    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        _db = new Daughterboard(slot);
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){
            _db->read_eeprom((ch<<4),14,calMem);
            if ((calMem[0]==0x18)&&(calMem[1]==0x43))
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    calInfo[i] = (uint32_t(calMem[2+3*i])<<16)+(uint32_t(calMem[3+3*i])<<8)+uint32_t(calMem[4+3*i]);
                }
                _smu  = new SMU(_db, ch, calInfo);
            }
            else _smu  = new SMU(_db, ch);
            //Sourcing voltage from AD5753, 10V BIP
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_10V_BIP);
            for (uint32_t i = 0; i < 65536; i+=256)
            {
                value = i;
                ret |= _smu->set_voltage(value);
                Serial.print(value);
                for (uint8_t n = 0; n < 10; n++)
                {
                    ret |= _smu->measure_source_current(&i_sense);
                    ret |= _smu->measure_sense_voltage(&v_sense);
                    ret |= _smu->measure_load_voltage(&v_load);
                    Serial.print(",");
                    Serial.print(i_sense);
                    Serial.print(",");
                    Serial.print(v_sense);
                    Serial.print(",");
                    Serial.print(v_load);
                }
                Serial.println("");
            }
            // ret |= _smu->set_voltage(0x8000);
            //Sourcing current from AD5753, 0-20mA
            ret |= _smu->select_current_mode(AD5753_DAC_CONFIG_RANGE_20mA_BIP);
            for (uint32_t i = 0; i < 65536; i+=256)
            {
                value = i;
                ret |= _smu->set_current(value);
                Serial.print(value);
                for (uint8_t n = 0; n < 10; n++)
                {
                    ret |= _smu->measure_source_current(&i_sense);
                    ret |= _smu->measure_sense_voltage(&v_sense);
                    ret |= _smu->measure_load_voltage(&v_load);
                    Serial.print(",");
                    Serial.print(i_sense);
                    Serial.print(",");
                    Serial.print(v_sense);
                    Serial.print(",");
                    Serial.print(v_load);
                }
                Serial.println("");
            }
            // ret |= _smu->set_current(0x8000);
            delayMicroseconds(200);
            delete(_smu);
        }
        }
        delete(_db);
    }
    }
    // ret |= scpi_rst();
    return ret;
}

exit_code Parser::scpi_calibrate(uint8_t slot, uint8_t ch, String value, bool isQuery)
{
    exit_code ret = SUCCESS;
    _db = new Daughterboard(slot);
    uint8_t calInfo[14];
    uint32_t data;
    if (isQuery){
        ret |= _db->read_eeprom((ch<<4),14,calInfo);
        for (uint8_t i = 0; i < 4; i++)
        {
            data = (uint32_t(calInfo[2+3*i])<<16)+(uint32_t(calInfo[3+3*i])<<8)+uint32_t(calInfo[4+3*i]);
            Serial.println(data);
        }
    }
    else{
        uint8_t idx;
        for (uint8_t i = 0; i < 4; i++)
        {
            idx = value.indexOf(',');
            if (idx>0) data = value.substring(0,idx).toInt();
            else 
            {   
                Serial.println("Input Invalid");
                return ret |= SCPI_FAULT;
            }
            value.remove(0,idx+1);
            calInfo[2+3*i] = uint8_t((data>>16)&0xFF);
            calInfo[3+3*i] = uint8_t((data>>8)&0xFF);
            calInfo[4+3*i] = uint8_t(data&0xFF);

            Serial.print(data);
            Serial.print(" ");
        }
        calInfo[0] = 0x18;
        calInfo[1] = 0x43;
        ret |= _db->write_eeprom((ch<<4),14,calInfo);
        Serial.println("Done");
    }   
    delete(_db);
    return ret;
}

exit_code Parser::scpi_calibrate_reset(void)
{
    exit_code ret = SUCCESS;
    uint8_t calMem[14];
    for (uint8_t idx = 0; idx < 14; idx++) calMem[idx] = 0x00;
    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        _db = new Daughterboard(slot);
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){
            ret |= _db->write_eeprom((ch<<4),14,calMem);
            // Serial.println(ch);
        }
        }
        delete(_db);
    }
    }
    Serial.println("Done");
    return ret;
}
#endif

exit_code Parser::scpi_occ(void)
{
    Serial.println("Occupancy:");
    for (uint8_t slot = 0; slot < 16; slot ++)
    {
        Serial.println(String(_occupancy[slot],BIN));
    }
}

exit_code Parser::scpi_config(uint8_t slot, uint8_t ch, String range, bool isQuery)
{
    exit_code ret = SUCCESS;
    _db = new Daughterboard(slot);
    _smu = new SMU(_db, ch);
    if (isQuery){
        uint8_t _range;
        uint8_t _mode;
        ret |= _smu->get_range_mode(&_range, &_mode);
        Serial.print(_mode);
        Serial.print(" ");
        Serial.println(_range);
    }
    else{
        if (range == RANGE_5V_UNI)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 5V UNI");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_5V_UNI);
        }
        else if (range == RANGE_10V_UNI)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 10V UNI");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_10V_UNI);
        }
        else if (range == RANGE_5V_BIP)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 5V UNI");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_5V_BIP);
        }
        else if (range == RANGE_10V_BIP)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 10V BIP");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_10V_BIP);
        }
        else if (range == RANGE_20mA_UNI)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 20mA UNI");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_20mA_UNI);
        }
        else if (range == RANGE_24mA_UNI)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 24mA UNI");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_24mA_UNI);
        }
        else if (range == RANGE_4_24mA)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 4 24mA");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_4_24mA_UNI);
        }
        else if (range == RANGE_20mA_BIP)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 20mA BIP");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_20mA_BIP);
        }
        else if (range == RANGE_24mA_BIP)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 24mA BIP");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_24mA_BIP);
        }
        else if (range == RANGE_22mA_OVR)
        {
            // Serial.println("Set Channel "+String(ch,DEC)+" mode to: 22mA OVR");
            Serial.println("DONE");
            ret |= _smu->select_voltage_mode(AD5753_DAC_CONFIG_RANGE_22mA_OVR);
        }
        else
        {
            Serial.println("Invalid Range");
            ret |= DAC_RANGE_FAULT;
        }
    }
    delete(_smu);
    delete(_db);
    return ret;
}

exit_code Parser::scpi_current(uint8_t slot, uint8_t ch, uint16_t value, bool isQuery)
{
    exit_code ret = SUCCESS;
    _db = new Daughterboard(slot);
    _smu = new SMU(_db, ch);
    uint8_t _range;
    uint8_t _dac_mode;
    ret |= _smu->get_range_mode(&_range, &_dac_mode);
    if (_dac_mode==SMU_CURRENT_MODE){
        if (isQuery){
            uint16_t _dac_code;
            ret |= _smu->get_dac_code(&_dac_code);
            Serial.println(_dac_code, DEC);
        }
        else{
            ret |= _smu->set_current(value);
            // Serial.println("Set channel " + String(ch,DEC) + "'s current to 0x" + String(value,HEX));
            Serial.println("DONE");
        }               
    }
    else{
        // Serial.println("Mode Error: Channel " + String(ch,DEC) + " is not in Current mode");
        Serial.println("Mode Error");
        ret |= SCPI_FAULT;
    }
    delete(_smu);
    delete(_db);
    return ret;
}

exit_code Parser::scpi_voltage(uint8_t slot, uint8_t ch, uint16_t value, bool isQuery)
{
    exit_code ret = SUCCESS;
    _db = new Daughterboard(slot);
    _smu = new SMU(_db, ch);
    uint8_t _range;
    uint8_t _dac_mode;
    ret |= _smu->get_range_mode(&_range, &_dac_mode);
    if (_dac_mode==SMU_VOLTAGE_MODE){
        if (isQuery){
            uint16_t _dac_code;
            ret |= _smu->get_dac_code(&_dac_code);
            Serial.println(_dac_code, DEC);
        }
        else{
            ret |= _smu->set_voltage(value);
            // Serial.println("Set channel " + String(ch,DEC) + "'s voltage to 0x" + String(value,HEX));
            Serial.println("DONE");
        }               
    }
    else{
        // Serial.println("Mode Error: Channel " + String(ch,DEC) + " is not in Voltage mode");
        Serial.println("Mode Error");
        ret |= SCPI_FAULT;
    }
    delete(_smu);
    delete(_db);
    return ret;
}

exit_code Parser::scpi_measure(uint8_t slot, uint8_t ch, String type)
{
    exit_code ret = SUCCESS;
    uint32_t data;
    uint16_t data16;
    _db = new Daughterboard(slot);
    _smu;
    uint8_t calMem[14];
    uint32_t calInfo[4];
    _db->read_eeprom((ch<<4),14,calMem);
    if ((calMem[0]==0x18)&&(calMem[1]==0x43))
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            calInfo[i] = (uint32_t(calMem[2+3*i])<<16)+(uint32_t(calMem[3+3*i])<<8)+uint32_t(calMem[4+3*i]);
        }
        _smu  = new SMU(_db, ch, calInfo);
    }
    else _smu  = new SMU(_db, ch);
        
    if (type == AD5753_REFIN)
    {
        ret |= _smu->read_debug_adc(0b00011, &data16);
        data = uint32_t(data16);
    }
    else if (type == AD5753_REFGND)
    {
        ret |= _smu->read_debug_adc(0b11000, &data16);
        data = uint32_t(data16);
    }
#if ADADC_COMPILE
    else if (type == AD5753_ISENSE)
    {
        ret |= smu->read_debug_adc1(&data16);
        data = uint32_t(data16);
    }
    else if (type == AD5753_LOADP)
    {
        ret |= smu->read_debug_adc2(&data16);
        data = uint32_t(data16);
    }
#endif
    else if (type == MAX11254_CURRENT)
    {
        ret |= _smu->measure_source_current(&data);
    }
    else if (type == MAX11254_SENSE_VOLTAGE)
    {
        ret |= _smu->measure_sense_voltage(&data);
    }
    else if (type == MAX11254_LOAD_VOLTAGE)
    {
        ret |= _smu->measure_load_voltage(&data);
    }
    else
    {
        Serial.println("Invalid measure type");
        ret |= MEAS_TYPE_FAULT;
    }
    if (ret == SUCCESS) Serial.println(data, DEC);
    delete(_smu);
    delete(_db);
    return ret;
}

#if SW_COMPILE
exit_code Parser::scpi_output_switch(uint8_t slot, uint8_t ch, String value, bool isQuery)
{
    exit_code ret = SUCCESS;
    Daughterboard* db = new Daughterboard(slot);
    SMU* smu = new SMU(db, ch);
    if (value == SW_INTERNAL){
        ret |= smu->switch_output_target(true);
        Serial.println("Ch "+String(ch,DEC)+", int res");
    } 
    else if (value == SW_EXTERNAL){
        ret |= smu->switch_output_target(false);
        Serial.println("Set Channel " + String(ch,DEC) + " to external load");
    }
    else
    {
        Serial.println("Invalid switch setting");
        ret |= SW_TYPE_FAULT;
    }
    delete(smu);
    delete(db);
    return ret;
}
#endif