/*
Parser.cpp - Configures the socket server for the controller board
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "Parser.h"


Parser::Parser(uint8_t* occupancy):
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
        if (cmd==SCPI_CALC){
            ret |= scpi_calibrate_rst();
            return ret;
        }
    }
    else{ //Funtional Command
        String header;
        String value;
        uint8_t ch;
        bool isQuery;
        cmd_decomp(cmd, &header, &value, &ch, &isQuery);
        //Check channel availablity
        if ( !(*_occupancy&BIT(ch)) )
        {
            #if SERIAL_DEBUG
                Serial.println("OCC ERR");
            #endif
            #if CONSOLE_DEBUG
                Console.println("OCC ERR");
            #endif
            return ret | SMU_UNAVAILABLE;
        }
        //Execute
        if (header==SCPI_CONF){
            ret |= scpi_config(ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CODE){
            ret |= scpi_set_code(ch, uint16_t(value.toInt()), isQuery);
            return ret;
        }
        if (header==SCPI_MEAS){
            ret |= scpi_measure(ch, value);
            return ret;
        }
        if (header==SCPI_SW){
            ret |= scpi_output_switch(ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CALS){
            ret |= scpi_calibrate_set(ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_SWP){
            ret |= scpi_sweep_ch(ch, value);
            return ret;
        }
    }
    #if SERIAL_DEBUG
        Serial.println("CMD ERR");
    #endif
    #if CONSOLE_DEBUG
        Console.println("CMD ERR");
    #endif
    return ret | SCPI_FAULT;
}

exit_code Parser::cmd_decomp(String cmd, String* header, String* value, uint8_t* ch, bool* isQuery)
{
    exit_code ret = SUCCESS;
    //cmd channel
    int8_t idx = cmd.indexOf(':');
    if (idx>0){
        *ch = cmd.substring(0,idx).toInt();
    }
    else{
        #if SERIAL_DEBUG
            Serial.println("IDX ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("IDX ERR");
        #endif
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
            #if SERIAL_DEBUG
                Serial.println("VALUE ERR");
            #endif
            #if CONSOLE_DEBUG
                Console.println("VALUE ERR");
            #endif
            ret |= SCPI_FAULT;
            return ret;
        }
    }
    //cmd value
    cmd.remove(0,idx+1);
    *value = cmd;

    #if SCPI_DEBUG and SERIAL_DEBUG
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
    #if SERIAL_DEBUG
        Serial.println("SMU SB YUN");
    #endif
    #if CONSOLE_DEBUG
        Console.println("SMU SB YUN");
    #endif
    return ret;
}

exit_code Parser::scpi_rst()
{
    #if SERIAL_DEBUG
        Serial.println("RST");
    #endif
    #if CONSOLE_DEBUG
        Console.println("RST");
    #endif
    exit_code ret = SUCCESS;

    *_occupancy = BIT(2)|BIT(3);

    ret |= comm_setup();
    ret |= controller_setup();
    ret |= board_setup(_occupancy);

    #if SERIAL_DEBUG
        if (ret == SUCCESS)
        {
            Serial.println("SETUP OK");
        }
        else
        {
            Serial.print("SETUP ERR:");
            Serial.println(ret, HEX);
        }
    #endif

    #if CONSOLE_DEBUG
        if (ret == SUCCESS)
        {
            Console.println("SETUP OK");
        }
        else
        {
            Console.print("SETUP ERR:");
            Console.println(ret, HEX);
        }
    #endif

    return ret;
}

exit_code Parser::scpi_calibrate_set(uint8_t ch, String value, bool isQuery)
{
    exit_code ret = SUCCESS;
    uint8_t calMem[16];
    if (isQuery){
        ret |= board_read_eeprom((ch<<4),16,calMem);
        #if SERIAL_DEBUG
            for (uint8_t i = 0; i < 16; i++)
            {
                Serial.print(String(calMem[i],HEX)+"-");
            }
            Serial.println();
        #endif
        #if CONSOLE_DEBUG
            for (uint8_t i = 0; i < 16; i++)
            {
                Console.print(String(calMem[i],HEX)+"-");
            }
            Console.println();
        #endif
    }
    else{
        uint8_t idx;
        for (uint8_t i = 2; i < 16; i++)
        {
            idx = value.indexOf(',');
            if (idx>0) calMem[i] = value.substring(0,idx).toInt();
            else 
            {
                #if SERIAL_DEBUG
                    Serial.println("CAL PARAM ERR");
                #endif
                return ret |= SCPI_FAULT;
            }
            value.remove(0,idx+1);
        }
        calMem[0] = 0x18;
        calMem[1] = 0x43;
        ret |= board_write_eeprom((ch<<4),16,calMem);
        #if SERIAL_DEBUG
            Serial.println("CAL PARAM SET");
        #endif
        #if CONSOLE_DEBUG
            Console.println("CAL PARAM SET");
        #endif
    }
    return ret;
}

exit_code Parser::scpi_calibrate_rst(void)
{
    exit_code ret = SUCCESS;
    uint8_t calMem[16];
    for (uint8_t idx = 0; idx < 16; idx++) calMem[idx] = 0x00;
        for (uint8_t ch = 0; ch<8; ch++){
        if ( *_occupancy&BIT(ch) ){
            ret |= board_write_eeprom((ch<<4),16,calMem);
        }
        }
    #if SERIAL_DEBUG
        Serial.println("CAL PARAM RST");
    #endif
    #if CONSOLE_DEBUG
        Console.println("CAL PARAM RST");
    #endif
    return ret;
}

exit_code Parser::scpi_config(uint8_t ch, String range, bool isQuery)
{
    exit_code ret = SUCCESS;
    if (isQuery){
        uint8_t _range;
        ret |= smu_get_range(ch, &_range);
        #if SERIAL_DEBUG
            Serial.println(_range);
        #endif
        #if CONSOLE_DEBUG
            Console.println(_range);
        #endif
    }
    else{
        if (range == MODE_VOLT)
        {
            ret |= smu_select_range(ch, AD5753_DAC_CONFIG_RANGE_5V_UNI);
            ret |= smu_set_dac_code(ch, 0x0);
        }
        else if (range == MODE_CURR)
        {
            ret |= smu_select_range(ch, AD5753_DAC_CONFIG_RANGE_20mA_UNI);
            ret |= smu_set_dac_code(ch, 0x0);
        }
        else
        {
            #if SERIAL_DEBUG
                Serial.println("RANGE ERR");
            #endif
            #if CONSOLE_DEBUG
                Console.println("RANGE ERR");
            #endif
            ret |= DAC_RANGE_FAULT;
            return ret;
        }
        #if SERIAL_DEBUG
            Serial.println("RANGE SET");
        #endif
        #if CONSOLE_DEBUG
            Console.println("RANGE SET");
        #endif
    }
    return ret;
}

exit_code Parser::scpi_set_code(uint8_t ch, uint16_t value, bool isQuery)
{
    exit_code ret = SUCCESS;
    if (isQuery){
        uint16_t _dac_code;
        ret |= smu_get_dac_code(ch, &_dac_code);
        Serial.println(_dac_code, DEC);
    }
    else{
        uint8_t _range;
        ret |= smu_get_range(ch, &_range);
        if(_range == AD5753_DAC_CONFIG_RANGE_5V_UNI)
        {
            if (value>MAX_CODE_V) value = MAX_CODE_V;
        }
        else if(_range == AD5753_DAC_CONFIG_RANGE_20mA_UNI)
        {
            if (value>MAX_CODE_I) value = MAX_CODE_I;
        }
        else
        {
            return ret | DAC_RANGE_FAULT;
        }
        ret |= smu_set_dac_code(ch, value);
        #if SERIAL_DEBUG
            Serial.println("DAC CODE SET");
        #endif
        #if CONSOLE_DEBUG
            Console.println("DAC CODE SET");
        #endif
    }               
    return ret;
}

exit_code Parser::scpi_measure(uint8_t ch, String param)
{
    exit_code ret = SUCCESS;
    uint32_t data,data_avg;
    String type;
    uint8_t n_rep;
    int8_t idx = param.indexOf(',');
    if (idx>0) type = param.substring(0,idx);
    param.remove(0,idx+1);
    n_rep = uint8_t(param.toInt());
    data_avg = 0;
    if (type == MEAS_VOLT)
    {
        for (uint8_t i = 0; i < n_rep; i++)
        {
            ret |= smu_measure_v(ch, &data);
            data_avg+=data;
        }
        data_avg/=n_rep;
    }
    else if (type == MEAS_CURR)
    {
        for (uint8_t i = 0; i < n_rep; i++)
        {
            ret |= smu_measure_i(ch, &data);
            data_avg+=data;
        }
        data_avg/=n_rep;
    }
    else
    {
        #if SERIAL_DEBUG
            Serial.println("MEAS TYPE ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("MEAS TYPE ERR");
        #endif
        return ret|MEAS_TYPE_FAULT;
    }

    if (ret == SUCCESS) 
    {
        #if SERIAL_DEBUG
            Serial.println(data_avg, HEX);
        #endif
        #if CONSOLE_DEBUG
            Console.println(data_avg, HEX);
        #endif
    }
    else
    {
        #if SERIAL_DEBUG
            Serial.println("MEAS ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("MEAS ERR");
        #endif
    }
    return ret;
}

exit_code Parser::scpi_output_switch(uint8_t ch, String value, bool isQuery)
{
    exit_code ret = SUCCESS;
    if (value == SW_INTERNAL){
        ret |= smu_switch_output_target(ch, true);
        #if SERIAL_DEBUG
            Serial.println("SW INT SET");
        #endif
        #if CONSOLE_DEBUG
            Console.println("SW INT SET");
        #endif
    } 
    else if (value == SW_EXTERNAL){
        ret |= smu_switch_output_target(ch, false);
        #if SERIAL_DEBUG
            Serial.println("SW EXT SET");
        #endif
        #if CONSOLE_DEBUG
            Console.println("SW EXT SET");
        #endif
    }
    else
    {
        #if SERIAL_DEBUG
            Serial.println("SW ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("SW ERR");
        #endif
        ret |= SW_TYPE_FAULT;
    }
    return ret;
}

exit_code Parser::scpi_sweep_ch(uint8_t ch, String param) //param = step,n_step,n_rep,
{
    exit_code ret = SUCCESS;

    uint16_t value,step;
    uint32_t i_sense, i_sense_avg, v_sense, v_sense_avg;
    uint8_t n_step,n_rep;

    int8_t idx = param.indexOf(',');
    if (idx>0) step = param.substring(0,idx).toInt();
    param.remove(0,idx+1);

    idx = param.indexOf(',');
    if (idx>0) n_step = param.substring(0,idx).toInt();
    param.remove(0,idx+1);

    idx = param.indexOf(',');
    if (idx>0) n_rep = param.substring(0,idx).toInt();
    param.remove(0,idx+1);

    uint16_t max_value = n_step*step;
    if(max_value>MAX_CODE_I) 
    {
        max_value = MAX_CODE_I;
        #if SERIAL_DEBUG
            Serial.println("MAX VALUE ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("MAX VALUE ERR");
        #endif
    }

    if ( *_occupancy&BIT(ch) ){
        ret |= smu_select_range(ch, AD5753_DAC_CONFIG_RANGE_20mA_UNI);
        for (uint32_t i = step; i <= max_value; i+=step)
        {
            value = i;
            ret |= smu_set_dac_code(ch, value);
            delayMicroseconds(100);
            #if SERIAL_DEBUG
                Serial.print(value);
                v_sense_avg = 0;
                for (uint8_t i = 0; i < n_rep; i++)
                {
                    ret |= smu_measure_v(ch, &v_sense);
                    v_sense_avg+=v_sense;
                }
                v_sense_avg/=n_rep;
                i_sense_avg = 0;
                for (uint8_t i = 0; i < n_rep; i++)
                {
                    ret |= smu_measure_i(ch, &i_sense);
                    i_sense_avg+=i_sense;
                }
                i_sense_avg/=n_rep;
                Serial.print(",");
                Serial.print(v_sense_avg);
                Serial.print(",");
                Serial.println(i_sense_avg);
            #endif
            #if CONSOLE_DEBUG
                Console.print(value);
                v_sense_avg = 0;
                for (uint8_t i = 0; i < n_rep; i++)
                {
                    ret |= smu_measure_v(ch, &v_sense);
                    v_sense_avg+=v_sense;
                }
                v_sense_avg/=n_rep;
                i_sense_avg = 0;
                for (uint8_t i = 0; i < n_rep; i++)
                {
                    ret |= smu_measure_i(ch, &i_sense);
                    i_sense_avg+=i_sense;
                }
                i_sense_avg/=n_rep;
                Console.print(",");
                Console.print(v_sense_avg);
                Console.print(",");
                Console.println(i_sense_avg);
            #endif
        }
        ret |= smu_set_dac_code(ch, 0x0);
        delayMicroseconds(200);
    }
    else
    {
        #if SERIAL_DEBUG
            Serial.println("OCC ERR");
        #endif
        #if CONSOLE_DEBUG
            Console.println("OCC ERR");
        #endif
    }
    return ret;
}