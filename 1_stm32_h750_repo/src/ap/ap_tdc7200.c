/*
 * ap_tdc7200.c
 *
 *  Created on: Jul 20, 2024
 *      Author: ksj10
 */



#include "ap_tdc7200.h"


ap_tdc7200_t ap_tdc7200_inst;


void apTdc7200Init(void)
{
    ap_tdc7200_inst.state_measure_call = true;
}

void apTdc7200Main(void)
{
    if(ap_tdc7200_inst.state_measure_call == true)
    {
        tdc7200Main();
    }
}
