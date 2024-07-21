/*
 * ap.c
 *
 *  Created on: Jul 17, 2024
 *      Author: ksj10
 */


#include "ap.h"


void apInit(void)
{
    apTdc7200Init();
}

void apMain(void)
{
	while(true)
	{
	    apTdc7200Main();
	}
}
