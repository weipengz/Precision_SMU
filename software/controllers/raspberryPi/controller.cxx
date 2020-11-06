/*
Globals.h - Global configuration variables
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

#include "controller.h"

exit_code controller_setup(void)
{
	wiringPiSetup();
	
	pinMode(CS0,OUTPUT);
	pinMode(CS1,OUTPUT);
	pinMode(CS2,OUTPUT);
	pinMode(CS3,OUTPUT);
	
	pinMode(SELECT0,OUTPUT);
	pinMode(SELECT1,OUTPUT);
	pinMode(SELECT2,OUTPUT);
	pinMode(SELECT3,OUTPUT);
	
	pinMode(DADCS,OUTPUT);
	pinMode(MONCS_N,OUTPUT);
	
	pinMode(BSENABLE_N,OUTPUT);
	pinMode(MONRST_N,OUTPUT);
	
	pinMode(POWERINT,INPUT);
	pinMode(FAULTINT,INPUT);
	
	pullUpDnControl(POWERINT,PUD_UP);
	pullUpDnControl(FAULTINT,PUD_UP);
	
	digitalWrite(CS0, LOW);
	digitalWrite(CS1, LOW);
	digitalWrite(CS2, LOW);
	digitalWrite(CS3, LOW);
	
	digitalWrite(SELECT0, LOW);
	digitalWrite(SELECT1, LOW);
	digitalWrite(SELECT2, LOW);
	digitalWrite(SELECT3, LOW);
	
	digitalWrite(DADCS, LOW);
	digitalWrite(MONCS_N, HIGH);

	digitalWrite(BSENABLE_N, HIGH);
	digitalWrite(MONRST_N, HIGH);

	#if STD_DEBUG
		printf("----Controller Loaded----\n");
	#endif
	
	return SUCCESS;
}

//~ void controller_close(void)
//~ {
    //~ bcm2835_close();
//~ }
