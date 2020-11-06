/*
csv.cxx - csv file save and load
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "csv.h"

Csvfile::Csvfile(char* filename, bool rw)
{
	if (rw) cf.open(filename, fstream::in);
	else cf.open(filename, fstream::out|fstream::trunc );
}

void Csvfile::save_a_line(uint16_t dac_code, uint32_t* data, uint8_t n_rep)
{
	cf << dac_code;
	for (int i = 0; i < n_rep*2; i++)
	{
		cf << "," << data[i];
	}
	cf << "\n";
}

int Csvfile::read_a_line(uint16_t* dac_code, uint32_t* data, uint8_t n_rep)
{
	int idx;
	string line;
	getline(cf, line);
	idx = line.find_first_of(",");
	*dac_code = stoi(line.substr(0,idx));
	line.erase(0,idx+1);
	for (int i = 0; i < n_rep*2; i++)
	{
		idx = line.find_first_of(",");
		data[i] = stoi(line.substr(0,idx));
		line.erase(0,idx+1);		
	}
	return 0;
}

void Csvfile::close()
{
	cf.close();
}
