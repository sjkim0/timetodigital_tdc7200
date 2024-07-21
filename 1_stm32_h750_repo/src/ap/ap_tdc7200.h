/*
 * ap_tdc7200.h
 *
 *  Created on: Jul 20, 2024
 *      Author: ksj10
 */

#ifndef AP_AP_TDC7200_H_
#define AP_AP_TDC7200_H_


#include "ap.h"


typedef struct
{
    bool state_measure_call;
}ap_tdc7200_t;


extern ap_tdc7200_t ap_tdc7200_inst;


void apTdc7200Init(void);
void apTdc7200Main(void);


#endif /* AP_AP_TDC7200_H_ */
