/*
parser.cxx - Parser
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "parser.h"
#include "backplane.h"
#include "dac.h"
#include "adc.h"
#include "eeprom.h"
#include "csv.h"

Parser::Parser(uint8_t* occupancy):
    _occupancy(occupancy)
{
}

exit_code Parser::parsing(string cmd)
{
    exit_code ret = SUCCESS;
    if (cmd[0]=='*') {//Common Command
        cmd.erase(0,1);
        if (cmd==SCPI_IDN){
            ret |= scpi_idn();
            return ret;
        }
        if (cmd==SCPI_SHDN){
            ret |= scpi_shdn();
            return ret;
        }
        if (cmd==SCPI_RST){
            ret |= scpi_rst();
            return ret;
        }
        if (cmd==SCPI_CAL){
            ret |= scpi_local_cal();
            return ret;
        }
        if (cmd==SCPI_CALC){
            ret |= scpi_cal_rst();
            return ret;
        }
        if (cmd==SCPI_OCC){
            ret |= scpi_occ();
            return ret;
        }
    }
    else{ //Funtional Command
        string header;
        string value;
        uint8_t ch;
        uint8_t slot;
        bool isQuery;
        cmd_decomp(cmd, &header, &value, &slot, &ch, &isQuery);
        //Check channel availablity
        if ( !(_occupancy[slot]&BIT(ch)) )
        {
			#if STD_DEBUG
				printf("Channel Error: Invalid slot or channel\n");
			#endif
            return ret | SMU_UNAVAILABLE;
        }
        //Execute
        if (header==SCPI_CONF){
            ret |= scpi_config(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CODE){
            ret |= scpi_dac_code(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CURR){
			ret |= scpi_current(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_VOLT){
            ret |= scpi_voltage(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_MEAS){
            ret |= scpi_measure(slot, ch, value);
            return ret;
        }
        if (header==SCPI_REFR){
            ret |= scpi_output_switch(slot, ch, value, isQuery);
            return ret;
        }
        if (header==SCPI_CALS){
            ret |= scpi_set_cal(slot, ch, value, isQuery);
            return ret;
        }
    }
    #if STD_DEBUG
		printf("Syntax Error: Invalid Command\n");
    #endif
    return ret | SCPI_FAULT;
}


exit_code Parser::cmd_decomp(string cmd, string* header, string* value, uint8_t* slot, uint8_t* ch, bool* isQuery)
{
    exit_code ret = SUCCESS;
    //cmd slot
    int8_t idx = cmd.find_first_of(":");
    if (idx>0){
		//~ printf("idx:%d\n",idx);
        *slot = stoi(cmd.substr(0,idx));
    }
    else{
		#if STD_DEBUG
			printf("Syntax Error: no valid slot\n");
        #endif
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.erase(0,idx+1);
    //cmd channel
    idx = cmd.find_first_of(":");
    if (idx>0){
        *ch = stoi(cmd.substr(0,idx));
    }
    else{
		#if STD_DEBUG
			printf("Syntax Error: no valid channel\n");
        #endif
        ret |= SCPI_FAULT;
        return ret;
    }
    cmd.erase(0,idx+1);
    //cmd header
    idx = cmd.find_first_of(":");
    if (idx>0){
        *header = cmd.substr(0,idx);
        *isQuery = false;
    }
    else{
        idx = cmd.find('?');
        if (idx>0){
            *header = cmd.substr(0,idx);
            *isQuery = true;
        }
        else{
			#if STD_DEBUG
				printf("Syntax Error: no valid value\n");
            #endif
            ret |= SCPI_FAULT;
            return ret;
        }
    }
    //cmd value
    cmd.erase(0,idx+1);
    *value = cmd;

    #if STD_DEBUG and SCPI_DEBUG
        printf("Slot: %d, Ch: %d, Header: %s, Value: %s, isQuery: %d\n", *slot, *ch, header->c_str(), value->c_str(), *isQuery);
    #endif

    return ret;
}

exit_code Parser::scpi_idn(){
	exit_code ret = SUCCESS;
	#if STD_DEBUG
		printf("IMEBUILD V2 Rev B\n");
	#endif

	uint16_t nTest = 100;
	uint8_t data_in[10] = {0,1,2,3,4,5,6,7,8,9};
	uint8_t data_out[10];
	uint8_t nFail = 0;
	//~ uint8_t mac_addr[6];

	bool passed;
	printf("Testing......\n");
	for (uint16_t i = 0; i < nTest; i++){
		passed = true;
		for (uint8_t slot = 0; slot < 16; slot++){
		if (_occupancy[slot]>0){
			for (uint8_t ch = 0; ch<8; ch++){
			if ( (_occupancy[slot]&BIT(ch)) > 0 ){
				ret |= eeprom_write(get_eeprom_dev(slot),(ch<<4),10,data_in);
			}
			}
		}
		}
		for (uint8_t slot = 0; slot < 16; slot++){
		if (_occupancy[slot]>0){
			for (uint8_t ch = 0; ch<8; ch++){
			if ( (_occupancy[slot]&BIT(ch)) > 0 ){
				ret |= eeprom_read(get_eeprom_dev(slot),(ch<<4),10,data_out);
				for (uint8_t idx = 0; idx < 10; idx++){
					if (data_out[idx]!=data_in[idx]) passed = false;
				}
			}
			}
		}
		}
		if (!passed) nFail++;
	}
	printf("EEPROM TEST: %d/%d\n", nFail, nTest);

	return ret;
}

exit_code Parser::scpi_shdn(){
    exit_code ret = SUCCESS;
	exit(0);
    return ret;
}

exit_code Parser::scpi_rst()
{
	#if STD_DEBUG
		printf("Resetting...\n");
    #endif
    exit_code ret = SUCCESS;
	ret |= imebuild_setup(_occupancy);
    return ret;
}

exit_code Parser::scpi_local_cal()
{
	exit_code ret = SUCCESS;
	#if STD_DEBUG
		printf("Self Calibrating...\n");
	#endif
	uint8_t n_rep = 3;
	uint16_t cal_step = 2048;
	scpi_cal_rst();
	ret |= cal_sweep(FOLDER_UNCAL, n_rep, cal_step);
	if (ret != SUCCESS)
	{
		printf("Sweep Error! 1\n");
		return FAILURE;
	}
	calc_cal_params(n_rep, cal_step);
	ret |= cal_sweep(FOLDER_CAL, n_rep, cal_step);
	if (ret != SUCCESS)
	{
		printf("Sweep Error! 2\n");
		return FAILURE;
	}
	return ret;
}

exit_code Parser::cal_sweep(string folder, uint8_t n_rep, uint16_t cal_step)
{//Calibration function
    exit_code ret = SUCCESS;
    uint16_t value = 0;
    uint8_t cal_mem[14];
    uint32_t cal_info[4];
    uint32_t data[n_rep*2];
    char filename[256];
    Csvfile* cf;

    int totalChCnt = 0;
    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){
			totalChCnt++;
        }
        }
    }
    }

	cout << "Performing sweeping of " << totalChCnt << " channels..." << endl;
	//~ int percent = 0;
	//~ int currentChNr = 0;
	uint8_t currentCh = 0;
    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){
            ret |= eeprom_read(get_eeprom_dev(slot),(ch<<4),14,cal_mem);
            if ((cal_mem[0]==0x18)&&(cal_mem[1]==0x43))
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    cal_info[i] = (uint32_t(cal_mem[2+3*i])<<16)+(uint32_t(cal_mem[3+3*i])<<8)+uint32_t(cal_mem[4+3*i]);
                }
            }
            else
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    cal_info[i] = 0x800000;
                }
	    }

		ret |= smu_setup(slot, ch);
            //Sourcing voltage from AD5753, 10V BIP
            sprintf(filename, "%sSlot%d_Ch%d_cal1.csv",folder.c_str(),slot,ch);
			cf = new Csvfile(filename, FILE_WRITE);
            ret |= smu_select_range(slot,ch,DAC_CONFIG_RANGE_10V_BIP);
            delayMicroseconds(100);
            for (uint32_t i = 0; i < 65536; i+=cal_step)
            {
                value = i;
				//~ #if STD_DEBUG
					//~ printf("%d",value);
				//~ #endif
                ret |= smu_set_dac_code(slot,ch,value);
                delayMicroseconds(100);
                for (uint8_t n = 0; n < n_rep; n++)
                {
                    ret |= smu_measure_i(slot, ch, data+n*2, cal_info);
                    delayMicroseconds(100);
                    ret |= smu_measure_v_sense(slot, ch, data+n*2+1, cal_info);
                    delayMicroseconds(100);
					//~ #if STD_DEBUG
						//~ printf(",%d,%d",i_sense,v_sense);
					//~ #endi
                }
				cf->save_a_line(value, data, n_rep);
				//~ #if STD_DEBUG
					//~ printf("\n");
				//~ #endif
				if (ret != SUCCESS) return FAILURE;
            }
            cf->close();
            delete(cf);
            delayMicroseconds(200);
            ret |= smu_set_dac_code(slot,ch,0x8000);

            //Sourcing current from AD5753, 0-20mA
            sprintf(filename, "%sSlot%d_Ch%d_cal2.csv",folder.c_str(),slot,ch);
			cf = new Csvfile(filename, FILE_WRITE);
            ret |= smu_select_range(slot,ch,DAC_CONFIG_RANGE_20mA_BIP);
            delayMicroseconds(100);
            for (uint32_t i = 0; i < 65536; i+=cal_step)
            {
                value = i;
				//~ #if STD_DEBUG
					//~ printf("%d",value);
				//~ #endif
                ret |= smu_set_dac_code(slot,ch,value);
                delayMicroseconds(100);
                for (uint8_t n = 0; n < n_rep; n++)
                {
                    ret |= smu_measure_i(slot, ch, data+n*2, cal_info);
                    delayMicroseconds(100);
                    ret |= smu_measure_v_sense(slot, ch, data+n*2+1, cal_info);
                    delayMicroseconds(100);
					//~ #if STD_DEBUG
					//~ printf(",%d,%d",i_sense,v_sense);
					//~ #endif
                }
                cf->save_a_line(value, data, n_rep);
				//~ #if STD_DEBUG
					//~ printf("\n");
				//~ #endif
				if (ret != SUCCESS) return FAILURE;
            }
            cf->close();
            delete(cf);
            delayMicroseconds(200);
            ret |= smu_set_dac_code(slot,ch,0x8000);
            currentCh++;
            printf("[Slot %d, Ch %d]: %d/%d\n", slot, ch, currentCh, totalChCnt);
        }
        }
    }
    }
    //cout << "\n";
    #if STD_DEBUG
		printf("Sweeping Done!\n");
	#endif
    return SUCCESS;
}

exit_code Parser::calc_cal_params(uint8_t n_rep, uint16_t cal_step)
{
	exit_code ret = SUCCESS;

	uint16_t n_sample = 65536/cal_step;
	double x[n_sample];
	double y[n_sample];

	double sumX,sumX2,sumY,sumXY;

	//Read in file
	string folder = FOLDER_UNCAL;
	char filename[256];
    Csvfile* cf;

    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){

			sprintf(filename, "%sSlot%d_Ch%d_cal1.csv",folder.c_str(),slot,ch);
			cf = new Csvfile(filename, FILE_READ);
			uint16_t dac_code;
			uint32_t data[n_rep*2];
			for (uint16_t i = 0; i < n_sample; i++)
			{
				cf->read_a_line(&dac_code, data, n_rep);
				y[i] = (dac_code-32768)*102.4+8388608;
				x[i] = 0;
				for (int n = 0; n < n_rep; n++)
				{
					x[i] += data[n*2+1];
				}
				x[i] /= n_rep;
			}
			sumX = 0;
			sumX2 = 0;
			sumY = 0;
			sumXY = 0;
			for (uint16_t i = 0; i < n_sample; i++)
			{
				sumX += x[i];
				sumX2 += x[i]*x[i];
				sumY += y[i];
				sumXY += x[i]*y[i];
			}
			cf->close();
			delete(cf);

			double coef = (n_sample*sumXY-sumX*sumY)/(n_sample*sumX2-sumX*sumX);
			uint32_t gain_v = uint32_t( (coef*10-9)*8388608 );
			uint32_t offset_v = uint32_t( (sumY-sumX*coef)/n_sample+8388608 );
			//~ printf("Slot%d,Ch%d,%f,%d,%d\n",slot,ch,coef,gain_v,offset_v);
			
			sprintf(filename, "%sSlot%d_Ch%d_cal2.csv",folder.c_str(),slot,ch);
			cf = new Csvfile(filename, FILE_READ);
			for (uint16_t i = 0; i < n_sample; i++)
			{
				cf->read_a_line(&dac_code, data, n_rep);
				y[i] = (dac_code-32768)*102.4*2.49/2.5+8388608;
				x[i] = 0;
				for (int n = 0; n < n_rep; n++)
				{
					x[i] += data[n*2];
				}
				x[i] /= n_rep;
			}
			sumX = 0;
			sumX2 = 0;
			sumY = 0;
			sumXY = 0;
			for (uint16_t i = 0; i < n_sample; i++)
			{
				sumX += x[i];
				sumX2 += x[i]*x[i];
				sumY += y[i];
				sumXY += x[i]*y[i];
			}
			cf->close();
			delete(cf);

			coef = (n_sample*sumXY-sumX*sumY)/(n_sample*sumX2-sumX*sumX);
			uint32_t gain_i = uint32_t( (coef*10-9)*8388608 );
			uint32_t offset_i = uint32_t( (sumY-sumX*coef)/n_sample+8388608 );
			//~ printf("Slot%d,Ch%d,%f,%d,%d\n",slot,ch,coef,gain_i,offset_i);
			
			uint8_t calInfo[14];
			
			calInfo[0] = 0x18;
			calInfo[1] = 0x43;
			
			calInfo[2] = uint8_t((gain_v>>16)&0xFF);
			calInfo[3] = uint8_t((gain_v>>8)&0xFF);
			calInfo[4] = uint8_t(gain_v&0xFF);
			
			calInfo[5] = uint8_t((offset_v>>16)&0xFF);
			calInfo[6] = uint8_t((offset_v>>8)&0xFF);
			calInfo[7] = uint8_t(offset_v&0xFF);

			calInfo[8] = uint8_t((gain_i>>16)&0xFF);
			calInfo[9] = uint8_t((gain_i>>8)&0xFF);
			calInfo[10] = uint8_t(gain_i&0xFF);
			
			calInfo[11] = uint8_t((offset_i>>16)&0xFF);
			calInfo[12] = uint8_t((offset_i>>8)&0xFF);
			calInfo[13] = uint8_t(offset_i&0xFF);

			ret |= eeprom_write(get_eeprom_dev(slot),(ch<<4),14,calInfo);
        }
        }
    }
    }
	#if STD_DEBUG
		printf("Calibration Params Set!\n");
	#endif
	return ret;
}

exit_code Parser::scpi_set_cal(uint8_t slot, uint8_t ch, string value, bool isQuery)
{
    exit_code ret = SUCCESS;
    uint8_t calInfo[14];
    uint32_t data;
    if (isQuery){
        ret |= eeprom_read(get_eeprom_dev(slot),(ch<<4),14,calInfo);
        for (uint8_t i = 0; i < 4; i++)
        {
            data = (uint32_t(calInfo[2+3*i])<<16)+(uint32_t(calInfo[3+3*i])<<8)+uint32_t(calInfo[4+3*i]);
            printf("%d\n",data);
        }
    }
    else{
        uint8_t idx;
        for (uint8_t i = 0; i < 4; i++)
        {
            idx = value.find_first_of(",");
            if (idx>0)
            {
				data = stoi(value.substr(0,idx));
			}
            else 
            {   
				#if STD_DEBUG
					printf("Input Invalid\n");
				#endif
                return ret |= SCPI_FAULT;
            }
            value.erase(0,idx+1);
            calInfo[2+3*i] = uint8_t((data>>16)&0xFF);
            calInfo[3+3*i] = uint8_t((data>>8)&0xFF);
            calInfo[4+3*i] = uint8_t(data&0xFF);
			#if STD_DEBUG
				printf("%d ",data);
			#endif
        }
        calInfo[0] = 0x18;
        calInfo[1] = 0x43;
        ret |= eeprom_write(get_eeprom_dev(slot),(ch<<4),14,calInfo);
        #if STD_DEBUG
			printf("Done\n");
		#endif
    }   
    return ret;
}

exit_code Parser::scpi_cal_rst(void)
{
    exit_code ret = SUCCESS;
    uint8_t calMem[14];
    for (uint8_t idx = 0; idx < 14; idx++) calMem[idx] = 0x00;
    for (uint8_t slot = 0; slot < 16; slot++){
    if (_occupancy[slot]>0){
        for (uint8_t ch = 0; ch<8; ch++){
        if ( (_occupancy[slot]&BIT(ch)) > 0 ){
            ret |= eeprom_write(get_eeprom_dev(slot),(ch<<4),14,calMem);
        }
        }
    }
    }
    #if STD_DEBUG
		printf("Calibration info reset!\n");
    #endif
    return ret;
}

exit_code Parser::scpi_occ(void)
{
    #if STD_DEBUG
		printf("Occupancy:\n");
		for (uint8_t slot = 0; slot < 16; slot ++)
		{
			printf("slot %d:",slot);
			for (uint8_t ch = 0; ch < 8; ch++)
			{
				if(_occupancy[slot]&BIT(ch)) printf("1");
				else printf("0");
			}
			printf("\n");
		}
    #endif
    return SUCCESS;
}

exit_code Parser::scpi_config(uint8_t slot, uint8_t ch, string range, bool isQuery)
{
    exit_code ret = SUCCESS;
    if (isQuery){
        uint8_t _range;
		ret |= smu_get_range(slot, ch, &_range);
        printf("%d\n",_range);
    }
    else{
        if (range == RANGE_5V_UNI)
        {
			#if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 5V UNI\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_5V_UNI);
        }
        else if (range == RANGE_10V_UNI)
        {
			#if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 10V UNI\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_10V_UNI);
        }
        else if (range == RANGE_5V_BIP)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 5V BIP\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_5V_BIP);
        }
        else if (range == RANGE_10V_BIP)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 10V BIP\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_10V_BIP);
        }
        else if (range == RANGE_20mA_UNI)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 20mA UNI\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_20mA_UNI);
        }
        else if (range == RANGE_24mA_UNI)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 24mA UNI\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_24mA_UNI);
        }
        else if (range == RANGE_4_24mA)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 4 to 24mA\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_4_24mA_UNI);
        }
        else if (range == RANGE_20mA_BIP)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 20mA BIP\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_20mA_BIP);
        }
        else if (range == RANGE_24mA_BIP)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: 24mA BIP\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_24mA_BIP);
        }
        else if (range == RANGE_22mA_OVR)
        {
            #if STD_DEBUG
				printf("Set Slot %d Channel %d's mode to: -1 to 22mA\n", slot, ch);
			#endif
            ret |= smu_select_range(slot, ch, DAC_CONFIG_RANGE_22mA_OVR);
        }
        else
        {
            #if STD_DEBUG
				printf("Invalid Range");
			#endif
            ret |= DAC_RANGE_FAULT;
        }
    }
    return ret;
}

exit_code Parser::scpi_dac_code(uint8_t slot, uint8_t ch, string value, bool isQuery)
{
    exit_code ret = SUCCESS;
	if (isQuery){
		uint16_t _dac_code;
		ret |= smu_get_dac_code(slot, ch, &_dac_code);
		printf("%d\n",_dac_code);
	}
	else{
		uint16_t dac_code = uint16_t(stoi(value));
		ret |= smu_set_dac_code(slot, ch, dac_code);
		printf("Set slot %d channel %d's DAC CODE to 0x%X\n", slot, ch, dac_code);
	}               
    return ret;
}

exit_code convert_value_to_code(double value, uint8_t range, uint16_t* dac_code)
{
	if ( (range==DAC_CONFIG_RANGE_5V_UNI) and (value >= 0) and (value <= 5) )
	{
		*dac_code = uint16_t(value/5*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_10V_UNI) and (value >= 0) and (value <= 10) )
	{
		*dac_code = uint16_t(value/10*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_5V_BIP) and (value >= -5) and (value <= 5) )
	{
		*dac_code = uint16_t((value+5)/10*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_10V_BIP) and (value >= -10) and (value <= 10) )
	{
		*dac_code = uint16_t((value+10)/20*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_20mA_UNI) and (value >= 0) and (value <= 20) )
	{
		*dac_code = uint16_t(value/20*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_24mA_UNI) and (value >= 0) and (value <= 24) )
	{
		*dac_code = uint16_t(value/24*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_4_24mA_UNI) and (value >= 4) and (value <= 24) )
	{
		*dac_code = uint16_t((value-4)/20*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_20mA_BIP) and (value >= -20) and (value <= 20) )
	{
		*dac_code = uint16_t((value+20)/40*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_24mA_BIP) and (value >= -24) and (value <= 24) )
	{
		*dac_code = uint16_t((value+24)/48*65535);
		return SUCCESS;
	}
	else if ( (range==DAC_CONFIG_RANGE_22mA_OVR) and (value >= -1) and (value <= 22) )
	{
		*dac_code = uint16_t((value+1)/23*65535);
		return SUCCESS;
	}
	else
	{
		return OVER_RANGE;
	}
}

exit_code Parser::scpi_current(uint8_t slot, uint8_t ch, string value, bool isQuery)
{
    exit_code ret = SUCCESS;
    uint8_t _range;
    ret |= smu_get_range(slot, ch, &_range);
    bool is_current_mode = DAC_CONFIG_CURRENT_MODE(_range);
    if (is_current_mode){
        if (isQuery){
            uint16_t _dac_code;
            ret |= smu_get_dac_code(slot, ch, &_dac_code);
            printf("%d\n",_dac_code);
        }
        else{
			double current = stod(value);
			uint16_t dac_code;
			if( convert_value_to_code(current, _range, &dac_code)==SUCCESS )
			{
				ret |= smu_set_dac_code(slot, ch, dac_code);
				printf("Set slot %d channel %d's current to %fmA (0x%X)\n", slot, ch, current, dac_code);
			}
			else
			{
				printf("Error: input value is out of range!\n");
			}
        }               
    }
    else{
        printf("Mode Error: Slot %d Channel %d is not in Current mode\n", slot, ch);
        ret |= SCPI_FAULT;
    }
    return ret;
}

exit_code Parser::scpi_voltage(uint8_t slot, uint8_t ch, string value, bool isQuery)
{
    exit_code ret = SUCCESS;
    uint8_t _range;
    ret |= smu_get_range(slot, ch, &_range);
    bool is_voltage_mode = !DAC_CONFIG_CURRENT_MODE(_range);
    if (is_voltage_mode){
        if (isQuery){
            uint16_t _dac_code;
            ret |= smu_get_dac_code(slot, ch, &_dac_code);
            printf("%d\n",_dac_code);
        }
        else{
			double voltage = stod(value);
			uint16_t dac_code;
			if( convert_value_to_code(voltage, _range, &dac_code)==SUCCESS )
			{
				ret |= smu_set_dac_code(slot, ch, dac_code);
				printf("Set slot %d channel %d's voltage to %fV (0x%X)\n", slot, ch, voltage, dac_code);
			}
			else
			{
				printf("Error: input value is out of range!\n");
			}
        }               
    }
    else{
        printf("Mode Error: Slot %d Channel %d is not in Voltage mode\n", slot, ch);
        ret |= SCPI_FAULT;
    }
    return ret;
}

exit_code convert_code_to_value(double* value, string type, uint32_t dac_code)
{
	if (type==DAC_REFIN)
	{
		*value = (double(dac_code)*2.75/4096);
		return SUCCESS;
	}
    else if (type==DAC_REFGND)
	{
		*value = (double(dac_code)*2.5/4096);
		return SUCCESS;
	}
    else if (type==DAC_ISENSE)
	{
		*value = (double(dac_code)*2.5/4096);
		return SUCCESS;
	}
    else if (type==DAC_VLOAD)
	{
		*value = (double(dac_code)*30/4096)-15;
		return SUCCESS;
	}
    else if ( type==ADC_CURRENT )
	{
		*value = (double(dac_code)/8388608-1)*2500/49.8;
		return SUCCESS;
	}
    else if ( type==ADC_LOAD_VOLTAGE or type==ADC_SENSE_VOLTAGE )
	{
		*value = (double(dac_code)/8388608-1)*25;
		return SUCCESS;
	}
	else
	{
		return MEAS_TYPE_FAULT;
	}
}

exit_code Parser::scpi_measure(uint8_t slot, uint8_t ch, string type)
{
    exit_code ret = SUCCESS;
    uint32_t data;
    uint16_t data16;
    uint8_t calMem[14];
    uint32_t calInfo[4];
    eeprom_read(get_eeprom_dev(slot),(ch<<4),14,calMem);
    if ((calMem[0]==0x18)&&(calMem[1]==0x43))
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            calInfo[i] = (uint32_t(calMem[2+3*i])<<16)+(uint32_t(calMem[3+3*i])<<8)+uint32_t(calMem[4+3*i]);
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            calInfo[i] = 0x800000;
        }
	}
        
    if (type == DAC_REFIN)
    {
        ret |= smu_read_debug_adc(slot, ch, 0b00011, &data16);
        data = uint32_t(data16);
    }
    else if (type == DAC_REFGND)
    {
        ret |= smu_read_debug_adc(slot, ch, 0b11000, &data16);
        data = uint32_t(data16);
    }
    else if (type == DAC_ISENSE)
    {
        ret |= smu_read_debug_adc(slot, ch, 0b10001, &data16);
        data = uint32_t(data16);
    }
    else if (type == DAC_VLOAD)
    {
        ret |= smu_read_debug_adc(slot, ch, 0b01100, &data16);
        data = uint32_t(data16);
    }
    else if (type == ADC_CURRENT)
    {
        ret |= smu_measure_i(slot, ch, &data, calInfo);
    }
    else if (type == ADC_SENSE_VOLTAGE)
    {
        ret |= smu_measure_v_sense(slot, ch, &data, calInfo);
    }
    else if (type == ADC_LOAD_VOLTAGE)
    {
        ret |= smu_measure_v_load(slot, ch, &data, calInfo);
    }
    else
    {
        printf("Invalid measure type\n");
        ret |= MEAS_TYPE_FAULT;
    }
    if (ret == SUCCESS)
    {
        double result; 
        convert_code_to_value(&result, type, data);
        printf("%f\n",result);
    } 
    return ret;
}

exit_code Parser::scpi_output_switch(uint8_t slot, uint8_t ch, string refr, bool isQuery)
{
    exit_code ret = SUCCESS;
    if (refr == SW_INTERNAL){
        ret |= smu_output_target(slot, ch, true);
        printf("Set slot %d channel%d's output target to internal resistor\n", slot, ch);
    } 
    else if (refr == SW_EXTERNAL){
        ret |= smu_output_target(slot, ch, false);
        printf("Set slot %d channel%d's output target to external resistor\n", slot, ch);
    }
    else
    {
        printf("Invalid switch setting\n");
        ret |= SW_TYPE_FAULT;
    }
    return ret;
}
