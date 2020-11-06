/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "backplane.h"
#include "controller.h"
#include "eeprom.h"
#include "monitor.h"
#include "dac.h"
#include "adc.h"

//Globals
exit_code imebuild_setup(uint8_t* occupancy)
{
    exit_code ret = SUCCESS;
    
    bool ch_is_exist, db_is_exist;
    uint8_t adc_fault, power_fault;
    uint8_t calInfo[2];
    for (uint8_t slot = 0; slot < 16; slot++) occupancy[slot] = 0x00;
    
    ret |= controller_setup();
    ret |= backplane_setup(0xFFFF);
    
    //~ uint8_t data;
    //~ ret |= adc_reg_read8(get_adc_dev(0,5),ADC_REG_SEQ,&data);
    //~ printf("%X\n",data);
    
    uint8_t smuNum = 0;
    for (uint8_t slot = 0; slot < 16; slot++)
    {
        ret |= daughterboard_is_exist(slot, &db_is_exist);
        if(db_is_exist)
        {
	    #if STD_DEBUG
		printf("Slot %02d: ",slot);
	    #endif   
            daughterboard_setup(slot,0xFF);
            daughterboard_monitor_diag(slot, &adc_fault, &power_fault);       
            for (uint8_t ch = 0; ch < 8; ch++)
            {
                //~ if( !(power_fault&BIT(ch)) && !(adc_fault&BIT(ch)) )
                //~ {
                    smu_is_exist(slot, ch, &ch_is_exist);
                    if(ch_is_exist)
                    {
			smuNum++;
                        occupancy[slot] |= BIT(ch);
                        ret |= smu_setup(slot, ch);
                        #if STD_DEBUG
                            printf("Ch %d: Good", ch);
                            eeprom_read(get_eeprom_dev(slot),(ch<<4),2,calInfo);
                            if ((calInfo[0]==0x18)&&(calInfo[1]==0x43)) 
                            {
                                printf(" Y");
                            }
                            else printf(" N");
                        printf(", ");
                        #endif
                    }
                    else
                    {
                        #if STD_DEBUG
                            printf("Ch %d: Bad, ", ch);
                        #endif
                        //~ uint16_t di_diag, an_diag;
                        //~ dac_reg_read(dac_dev, DAC_REG_DIGITAL_DIAG_RESULTS, &di_diag);
                        //~ dac_reg_read(dac_dev, DAC_REG_ANALOG_DIAG_RESULTS, &an_diag);
                        //~ printf("Diag Result: %X, %X\n",di_diag,an_diag);
                    }
                //~ }
                //~ else
                //~ {
                    //~ //fault handling
		    //~ #if STD_DEBUG
			//~ printf("Ch %d: Fault, ", ch);
		    //~ #endif
                //~ }
            }
            #if STD_DEBUG
		printf("\n");
	    #endif
            daughterboard_setup(slot,occupancy[slot]);
        }
        else
        {
	    #if STD_DEBUG
		    printf("Slot %02d is not used\n",slot); 
	    #endif
	}
    }
    uint16_t opy_db = 0x0000;
    for (uint8_t slot = 0; slot < 16; slot++)
        if (occupancy[slot]>0) opy_db |= BIT(slot);
    
    occupancy[8] = occupancy[8] & 0b11110111;
    backplane_setup(opy_db);	

    #if STD_DEBUG
        if (ret == SUCCESS)
        {
            printf("---> Setup finished successfully. Found %d SMU(s)\n", smuNum);
        }
        else
        {
            printf("---> Setup finished with error: %X\n",ret);
        }
    #endif
    return ret;
}

//Backplane
exit_code backplane_setup(uint16_t opy_db)
{
    exit_code ret = SUCCESS;
    /* Backplane monitors */
    uint8_t dataMSB = uint8_t((opy_db>>8)&0xFF);
    uint8_t dataLSB = uint8_t(opy_db&0xFF);
    // Reset monitor chips
    ret |= monitor_bp_reset();
    // Enable hardware select
    spi_device dev = get_bp_mon_dev();
    ret |= monitor_iocon_cfg(dev, 0x0, 0xE8);
    ret |= monitor_iocon_cfg(dev, 0x1, 0xE8);
    // Fault Monitor setup
    ret |= monitor_setup(dev, 0x0, dataLSB, dataMSB);
    // Power-good (PGOOD) Monitor setup
    ret |= monitor_setup(dev, 0x1, dataLSB, dataMSB);
    return ret;
}


//Daughterboard
exit_code daughterboard_is_exist(uint8_t slot, bool* db_is_exist)
{
    exit_code ret = SUCCESS;
    uint8_t mac_addr[6];
    ret |= eeprom_read(get_eeprom_dev(slot), 0xFA, 6, mac_addr);
    *db_is_exist = (mac_addr[0]==0x80) and (mac_addr[1]==0x1F) and (mac_addr[2]==0x12);
    //~ printf("%02X-%02X-%02X-%02X-%02X-%02X :",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
    return ret;
}

exit_code daughterboard_setup(uint8_t slot, uint8_t opy_ch)
{
    exit_code ret = SUCCESS;
    /* Backplane monitors */
    // Reset monitor chips
    ret |= monitor_db_reset(slot);
    // Enable hardware select
    spi_device dev = get_db_mon_dev(slot);
    ret |= monitor_iocon_cfg(dev, 0x00, 0xAA);//0xA8
    // Fault Monitor setup
    ret |= monitor_setup(dev, 0x00, opy_ch, opy_ch);
    return ret;
}

exit_code daughterboard_monitor_diag(uint8_t slot, uint8_t* adc_fault, uint8_t* power_fault)
{
    exit_code ret = SUCCESS;
    spi_device dev = get_db_mon_dev(slot);
    ret |= monitor_reg_read(dev, 0x00, MON_REG_INTFA, adc_fault);
    ret |= monitor_reg_read(dev, 0x00, MON_REG_INTFB, power_fault);
    return ret;
}


//SMU
exit_code smu_is_exist(uint8_t slot, uint8_t ch, bool* exist)
{
    exit_code ret = SUCCESS;
    uint16_t chip_id;
    ret |= dac_reg_read(get_dac_dev(slot, ch), DAC_REG_CHIP_ID, &chip_id);
    *exist = ((chip_id!=0xFFFF) && (chip_id!=0x0000));
    //~ printf("chip_id:%d\n",chip_id);
    return ret;
}

exit_code smu_setup(uint8_t slot, uint8_t ch)
{
    exit_code ret = SUCCESS;
    
    spi_device adc_dev = get_adc_dev(slot,ch);
    spi_device dac_dev = get_dac_dev(slot,ch);
    
    ret |= dac_startup_sequence(dac_dev);
    ret |= adc_setup(adc_dev);
    ret |= adc_calibrate(adc_dev, 0b00);
    
    ret |= smu_reference_switch(slot,ch,SW_INTERNAL);
    ret |= smu_output_enable(slot,ch,true);
	    
    ret |= dac_set_new_range(dac_dev, DEFAULT_RANGE);
    ret |= adc_prepare_convert(adc_dev, 1);

    return ret;
}

exit_code smu_output_enable(uint8_t slot, uint8_t ch, bool ENABLE)
{
	exit_code ret = SUCCESS;
	spi_device dac_dev = get_dac_dev(slot,ch);
	ret |= dac_reg_write_mask(dac_dev,
		DAC_REG_GPIO_DATA,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MSK,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MODE(!ENABLE)
		);
	delayMicroseconds(10);
	return ret;
}

exit_code smu_reference_switch(uint8_t slot, uint8_t ch, bool REFRES)
{
	exit_code ret = SUCCESS;
	spi_device dac_dev = get_dac_dev(slot,ch);
	ret |= dac_reg_write_mask(dac_dev,
		DAC_REG_GPIO_DATA,
		DAC_REG_GPIO_DATA_GPO_1_WRITE_MSK,
		DAC_REG_GPIO_DATA_GPO_1_WRITE_MODE(REFRES)
		);
	delayMicroseconds(10);
	return ret;
}

exit_code smu_output_target(uint8_t slot, uint8_t ch, bool REFRES)
{
    exit_code ret = SUCCESS;
    spi_device dac_dev = get_dac_dev(slot,ch);
    uint16_t current_dac_code;
    
    ret |= dac_pause_output(dac_dev, &current_dac_code);
    delayMicroseconds(200);
	ret |= dac_reg_write_mask(dac_dev,
		DAC_REG_GPIO_DATA,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MSK,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MODE(true)
		);
    delayMicroseconds(200);
	ret |= dac_reg_write_mask(dac_dev,
		DAC_REG_GPIO_DATA,
		DAC_REG_GPIO_DATA_GPO_1_WRITE_MSK,
		DAC_REG_GPIO_DATA_GPO_1_WRITE_MODE(REFRES)
		);
    delayMicroseconds(200);
	ret |= dac_reg_write_mask(dac_dev,
		DAC_REG_GPIO_DATA,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MSK,
		DAC_REG_GPIO_DATA_GPO_0_WRITE_MODE(false)
		);
    delayMicroseconds(200);
    
    uint16_t dac_config_reg;
    ret |= dac_reg_read(dac_dev, DAC_REG_DAC_CONFIG, &dac_config_reg);
    uint8_t range = uint8_t(dac_config_reg & 0xF);  
    
    ret |= dac_set_dac_code(dac_dev, range, current_dac_code);
    return ret;	
}

// Source Functions
exit_code smu_select_range(uint8_t slot, uint8_t ch, uint8_t range)
{
	/* Sets the DAC to voltage mode.
     * Side effects: if it was in current mode, it will set the output to 0V.
     */
    exit_code ret = SUCCESS;
    spi_device dac_dev = get_dac_dev(slot,ch);
    ret |= dac_set_new_range(dac_dev, range);
    return ret;
}

exit_code smu_get_range(uint8_t slot, uint8_t ch, uint8_t* range)
{
	/* Sets the DAC to voltage mode.
     * Side effects: if it was in current mode, it will set the output to 0V.
     */
    exit_code ret = SUCCESS;
    spi_device dac_dev = get_dac_dev(slot,ch);
    uint16_t dac_config_reg;
    ret |= dac_reg_read(dac_dev, DAC_REG_DAC_CONFIG, &dac_config_reg);
    *range = uint8_t(dac_config_reg & 0xF); 
    return ret;
}

exit_code smu_set_dac_code(uint8_t slot, uint8_t ch, uint16_t dac_code)
{
    exit_code ret = SUCCESS;
    spi_device dac_dev = get_dac_dev(slot,ch);

    uint16_t dac_config_reg;
    ret |= dac_reg_read(dac_dev, DAC_REG_DAC_CONFIG, &dac_config_reg);
    delayMicroseconds(10);
    uint8_t range = uint8_t(dac_config_reg & 0xF);
    //~ printf("dac config reg: %04X\n",dac_config_reg);
    //~ printf("range: %d\n",range);
    ret |= dac_set_dac_code(dac_dev, range, dac_code);
    return ret;
}

exit_code smu_get_dac_code(uint8_t slot, uint8_t ch, uint16_t* dac_code)
{
    exit_code ret = SUCCESS;
    spi_device dac_dev = get_dac_dev(slot,ch);
    ret |= dac_reg_read(dac_dev, DAC_REG_DAC_INPUT, dac_code);
    return ret;
}

exit_code smu_measure_v_sense(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info)
{
    exit_code ret = SUCCESS;
    spi_device adc_dev = get_adc_dev(slot,ch);
	
    uint32_t adc_code_raw;
    ret |= adc_convert(adc_dev, 2, 0b0111, &adc_code_raw);
    *adc_code = uint32_t( (float(cal_info[0])/0x800000*0.1+0.9)*int32_t(adc_code_raw)+cal_info[1]-0x800000);
    return ret;
}

exit_code smu_measure_v_load(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info)
{
    exit_code ret = SUCCESS;
    spi_device adc_dev = get_adc_dev(slot,ch);
	
    uint32_t adc_code_raw;
    ret |= adc_convert(adc_dev, 1, 0b0111, &adc_code_raw);
    *adc_code = uint32_t( (float(cal_info[0])/0x800000*0.1+0.9)*int32_t(adc_code_raw)+cal_info[1]-0x800000);
    return ret;
}

exit_code smu_measure_i(uint8_t slot, uint8_t ch, uint32_t* adc_code, uint32_t* cal_info)
{
    exit_code ret = SUCCESS;
    spi_device adc_dev = get_adc_dev(slot,ch);
    
    uint32_t adc_code_raw;
    ret |= adc_convert(adc_dev, 3, 0b0111, &adc_code_raw);
    *adc_code = uint32_t( (float(cal_info[2])/0x800000*0.1+0.9)*int32_t(adc_code_raw)+cal_info[3]-0x800000);
    return ret;
}

exit_code smu_read_debug_adc(uint8_t slot, uint8_t ch, uint8_t ip_addr, uint16_t* adc_data)
{
    spi_device dac_dev = get_dac_dev(slot,ch);	
    return dac_adc_single_conversion(dac_dev, ip_addr, adc_data);
}

//~ exit_code smu_get_range_mode(uint8_t* range, uint8_t* dac_mode);
//~ exit_code smu_get_dac_code(uint16_t* dac_code);
//~ exit_code smu_diagonse();

