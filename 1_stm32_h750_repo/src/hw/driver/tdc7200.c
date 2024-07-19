/*
 * tdc7200.c
 *
 *  Created on: Jul 19, 2024
 *      Author: ksj10
 */



#include "tdc7200.h"
#include "spi.h"


tdc7200_t tdc7200_inst;


void tdc7200Init(void)
{
	/* TODO:
	 *
	 * @ 0. Enable TDC
	 * @ 1. Read All Register
	 */

	// @ 0. Enable TDC
	HAL_GPIO_WritePin(OEN_GPIO_Port, OEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
	delay(100);
	// @ 1. Read All Register
	int start_index = ENUM_TDC7200_CONFIG_1;
	int stop_index = ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L;
	for(int i = start_index; i <= stop_index; i++)
	{
		HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, &tdc7200_inst.reg_1_byte_data[i], 1, 0xFFFF);
		HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
	}
	start_index = ENUM_TDC7200_TIME_1 - ENUM_TDC7200_TIME_1;
	stop_index = ENUM_TDC7200_CALIBRATION_2 - ENUM_TDC7200_TIME_1;
	for(int i = start_index; i <= stop_index; i++)
	{
		HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, tdc7200_inst.reg_3_byte_data[i], 3, 0xFFFF);
		HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
	}
}
