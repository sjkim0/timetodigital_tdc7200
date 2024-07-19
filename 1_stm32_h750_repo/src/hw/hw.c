/*
 * hw.c
 *
 *  Created on: Jul 17, 2024
 *      Author: ksj10
 */


#include "hw.h"


void hwInit(void)
{
	bspInit();

	MX_GPIO_Init();
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	tdc7200Init();
}
