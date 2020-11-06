/*
SMUBoard.h - Firmware for Backplane REV A
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "SMUBoard.h"


/* Board */
spi_device get_mon_rst_dev()
{
    #define CS_NUM 4

    uint8_t cs_pin_list[CS_NUM] = {
        CS3,
        CS2,
        CS1,
        CS0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        HIGH,
        HIGH,
        LOW
    };

    uint8_t cs_fast_pin = BS;
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

spi_device get_mon_dev()
{
    #define CS_NUM 4

    uint8_t cs_pin_list[CS_NUM] = {
        CS3,
        CS2,
        CS1,
        CS0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        LOW,
        LOW,
        HIGH
    };

    uint8_t cs_fast_pin = BS;
    uint8_t cs_fast_pin_state = LOW;

    struct spi_device ret;
    ret.cs_pin_num = CS_NUM;
    copy(cs_pin_list, ret.cs_pin_list, CS_NUM);
    copy(cs_pin_states, ret.cs_pin_states, CS_NUM);
    ret.cs_fast_pin = cs_fast_pin;
    ret.cs_fast_pin_state = cs_fast_pin_state;

    #undef CS_NUM

    return ret;
}

spi_device get_eeprom_dev()
{
    #define CS_NUM 4

    uint8_t cs_pin_list[CS_NUM] = {
        CS3,
        CS2,
        CS1,
        CS0
    };

    uint8_t cs_pin_states[CS_NUM] = {
        HIGH,
        LOW,
        LOW,
        LOW
    };

    uint8_t cs_fast_pin = BS;
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

exit_code board_mon_setup(uint8_t opy)
{
    exit_code ret = SUCCESS;

    ret |= board_mon_rst();
    ret |= mcp23s17_iocon_cfg(get_mon_dev(), 0x00, 0xAA);
    ret |= mcp23s17_setup(get_mon_dev(), 0x00, opy, opy);

    return ret;
}

exit_code board_mon_rst()
{
    exit_code ret = SUCCESS;
    spi_device dev = get_mon_rst_dev();

    ss_select(dev);
    ss_deselect(dev);

    // #if SERIAL_DEBUG
    //     Serial.println("MON RST");
    // #endif
    return ret;
}

exit_code board_read_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_dev();
    uint8_t _data[n_bytes];
    do
    {
        ret |= mc24aa02_read_mem_addr(dev, addr, n_bytes, data);
        ret |= mc24aa02_read_mem_addr(dev, addr, n_bytes, _data);
    } while (!compare_data(n_bytes, data, _data));

    return SUCCESS;
}

exit_code board_write_eeprom(uint8_t addr, uint8_t n_bytes, uint8_t* data)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_eeprom_dev();
    uint8_t _data[n_bytes];
    do
    {
        ret |= mc24aa02_write_mem_addr(dev, addr, n_bytes, data);
        delayMicroseconds(500);
        uint8_t estatus;
        do
        {
            ret |= mc24aa02_read_status(dev, &estatus);
            delayMicroseconds(500);
        }
        while ( (estatus&0x01)==0x01 );
        ret |= mc24aa02_read_mem_addr(dev, addr, n_bytes, _data);
    } while (!compare_data(n_bytes, data, _data));

    return SUCCESS;
}

exit_code board_setup(uint8_t* occupancy)
{
    exit_code ret = SUCCESS;
    bool smuIsExist;
    uint8_t smuNum = 0;

    uint8_t calMem[16];
    for (uint8_t ch = 0; ch < 8; ch++){
    if (*occupancy&BIT(ch)){
        ret |= smu_is_exist(ch, &smuIsExist);
        if (smuIsExist)
        {
            smuNum++;
            ret |= smu_setup(ch);
            ret |= smu_select_range(ch, AD5753_DEFAULT_RANGE);
            ret |= smu_prepare_convert(ch, true, 1);
            ret |= smu_set_dac_code(ch, 0x0);

            ret |= board_read_eeprom((ch<<4),16,calMem);
            #if SERIAL_DEBUG
                Serial.print("CH "+String(ch,DEC)+":");
                if( (calMem[0]==0x18) && (calMem[1]==0x43) )
                {
                    for (uint8_t i = 2; i < 16; i++)
                    {
                        Serial.print(String(calMem[i],HEX)+"-");
                    }
                }
                else
                {
                    Serial.print("NO CAL");
                }
                Serial.println();
            #endif
            #if CONSOLE_DEBUG
                Console.print("CH "+String(ch,DEC)+":");
                if( (calMem[0]==0x18) && (calMem[1]==0x43) )
                {
                    for (uint8_t i = 2; i < 16; i++)
                    {
                        Console.print(String(calMem[i],HEX)+"-");
                    }
                }
                else
                {
                    Console.print("NO CAL");
                }
                Console.println();
            #endif
        }
        else
        {
            *occupancy &= (~BIT(ch));
            #if SERIAL_DEBUG
                Serial.println("CH "+String(ch,DEC)+" BAD");
            #endif
        }
    }
    }
    ret |= board_mon_setup(*occupancy);

    return ret;
}

/* Channel */
spi_device get_adc_dev(uint8_t ch)
{
    #define CS_NUM 5

    uint8_t cs_pin_list[CS_NUM] = {
        CS3,
        CS2,
        CS1,
        CS0,
        DADCS
    };

    uint8_t cs_pin_states[CS_NUM] = {
        LOW,
        BIT2_FILT(ch),
        BIT1_FILT(ch),
        BIT0_FILT(ch),
        LOW
    };

    uint8_t cs_fast_pin = BS;
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

spi_device get_dac_dev(uint8_t ch)
{
    #define CS_NUM 5

    uint8_t cs_pin_list[CS_NUM] = {
        CS3,
        CS2,
        CS1,
        CS0,
        DADCS
    };

    uint8_t cs_pin_states[CS_NUM] = {
        LOW,
        BIT2_FILT(ch),
        BIT1_FILT(ch),
        BIT0_FILT(ch),
        HIGH
    };

    uint8_t cs_fast_pin = BS;
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


exit_code smu_output_switch(uint8_t ch, bool REFRES)
{
    return ad5753_reg_write_mask(
        get_dac_dev(ch),
        AD5753_REG_GPIO_DATA,
        AD5753_REG_GPIO_DATA_GPO_1_WRITE_MSK,
        AD5753_REG_GPIO_DATA_GPO_1_WRITE_MODE(REFRES)
        );
}

exit_code smu_output_enable(uint8_t ch, bool ENABLE)
{
    return ad5753_reg_write_mask(
        get_dac_dev(ch),
        AD5753_REG_GPIO_DATA,
        AD5753_REG_GPIO_DATA_GPO_0_WRITE_MSK,
        AD5753_REG_GPIO_DATA_GPO_0_WRITE_MODE(!ENABLE)
        );
}

exit_code smu_switch_output_target(uint8_t ch, bool REFRES)
{
    exit_code ret = SUCCESS;
    uint16_t current_dac_code;
    ret |= ad5753_pause_output(get_dac_dev(ch), &current_dac_code);
    delayMicroseconds(200);
    ret |= smu_output_enable(ch, false);
    delayMicroseconds(200);
    ret |= smu_output_switch(ch, REFRES);
    delayMicroseconds(200);
    ret |= smu_output_enable(ch, true);
    delayMicroseconds(200);
    ret |= smu_set_dac_code(ch, current_dac_code);
    return ret;
}

// Source Functions
exit_code smu_select_range(uint8_t ch, uint8_t range)
{
    exit_code ret = SUCCESS;
    ret |= ad5753_set_new_range(get_dac_dev(ch), range);
    return ret;
}

exit_code smu_set_dac_code(uint8_t ch, uint16_t dac_code)
{
    exit_code ret = SUCCESS;
    uint16_t dac_config_reg;
    ret |= ad5753_reg_read(get_dac_dev(ch), AD5753_REG_DAC_CONFIG, &dac_config_reg);
    uint8_t _range = uint8_t(dac_config_reg & 0xF);
    ret |= ad5753_set_dac_code(get_dac_dev(ch), _range, dac_code);
    return ret;
}

exit_code smu_prepare_convert(uint8_t ch, bool pga, uint8_t gain)
{
    if(pga) return max11254_prepare_convert(get_adc_dev(ch), gain);
    else return max11254_prepare_convert(get_adc_dev(ch), 0);
}

exit_code smu_measure_i(uint8_t ch, uint32_t* adc_code)
{
    exit_code ret = SUCCESS;
    ret |= max11254_convert(get_adc_dev(ch), 3, 0b0010, adc_code);
    return ret;
}

exit_code smu_measure_v(uint8_t ch, uint32_t* adc_code)
{
    exit_code ret = SUCCESS;
    ret |= max11254_convert(get_adc_dev(ch), 2, 0b0010, adc_code);
    return ret;
}

exit_code smu_get_range(uint8_t ch, uint8_t* range)
{
    exit_code ret = SUCCESS;
    uint16_t dac_config_reg;
    ret |= ad5753_reg_read(get_dac_dev(ch), AD5753_REG_DAC_CONFIG, &dac_config_reg);
    *range = uint8_t(dac_config_reg & 0xF);
    return ret;
}

exit_code smu_get_dac_code(uint8_t ch, uint16_t* dac_code)
{
    exit_code ret = SUCCESS;
    ret |= ad5753_reg_read(get_dac_dev(ch), AD5753_REG_DAC_INPUT, dac_code);
    return ret;
}

exit_code smu_is_exist(uint8_t ch, bool* exist)
{
    exit_code ret = SUCCESS;
    uint16_t chip_id;
    ret |= ad5753_reg_read(get_dac_dev(ch), AD5753_REG_CHIP_ID, &chip_id);
    *exist = ((chip_id!=0xFFFF) && (chip_id!=0x0000));
    // Serial.print("chip_id:"+String(chip_id,HEX)+" ");
    return ret;
}

exit_code smu_setup(uint8_t ch)
{
    exit_code ret = SUCCESS;
    // DAC Startup
    ret |= ad5753_startup_sequence(get_dac_dev(ch));
    // ADC Startup
    ret |= max11254_setup(get_adc_dev(ch));
    ret |= max11254_calibrate(get_adc_dev(ch), 0b00);
    // Output switches
    ret |= smu_output_switch(ch, true);
    ret |= smu_output_enable(ch, true);
    return ret;
}