/*
Controller.h - Backplane controller functionality
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef Controller_h
#define Controller_h

#include "Globals.h"

#define BS            7
#define CS0           3
#define CS1           4
#define CS2           5
#define CS3           6
#define DADCS         11
#define POWEROUT      9
#define FAULTOUT      8
#define LDAC          10
// #define MONITORCS_N     
// #define MONITORRST_N    


exit_code controller_setup(void);

#endif
