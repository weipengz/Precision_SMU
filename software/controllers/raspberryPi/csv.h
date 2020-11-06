/*
csv.h v - csv file save and load
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

#define FILE_READ  true
#define FILE_WRITE false

using namespace std;

class Csvfile
{
public:
    Csvfile(char* filename, bool rw);
    void save_a_line(uint16_t dac_code, uint32_t* data, uint8_t n_rep);
    int read_a_line(uint16_t* dac_code, uint32_t* data, uint8_t n_rep);
    void close();
private:
    fstream cf;
};


