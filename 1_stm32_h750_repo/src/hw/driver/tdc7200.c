/*
 * tdc7200.c
 *
 *  Created on: Jul 19, 2024
 *      Author: ksj10
 */



#include "tdc7200.h"
#include "spi.h"


tdc7200_t tdc7200_inst;

static void tdc7200ReadSpi(uint8_t addr, uint8_t *rx_buff);


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
	while(true)
	{
		int start_index = ENUM_TDC7200_CONFIG_1;
		int stop_index = ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L;
		for(int i = start_index; i <= stop_index; i++)
		{
			tdc7200ReadSpi(i, &tdc7200_inst.reg_1_byte_data[i]);
			delay(100);
		}
		start_index = ENUM_TDC7200_TIME_1 - ENUM_TDC7200_TIME_1;
		stop_index = ENUM_TDC7200_CALIBRATION_2 - ENUM_TDC7200_TIME_1;
		for(int i = start_index; i <= stop_index; i++)
		{
            tdc7200ReadSpi(i + ENUM_TDC7200_TIME_1, tdc7200_inst.reg_3_byte_data[i]);
			delay(100);
		}
	}
}

static void tdc7200ReadSpi(uint8_t addr, uint8_t *rx_buff)
{
	uint8_t tx_buff_byte[2] = {addr, };
	uint8_t tx_buff_3_byte[4] = {addr, };
    uint8_t rx_buff_byte[2] = {0, };
    uint8_t rx_buff_3_byte[4] = {0, };
	int length = 0;
	int one_byte_data_spi_length = 2;
	int three_byte_data_spi_length = 4;

    HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_RESET);
	switch(addr)
	{
        case ENUM_TDC7200_CONFIG_1:
        case ENUM_TDC7200_CONFIG_2:
        case ENUM_TDC7200_INT_STATUS:
        case ENUM_TDC7200_INT_MASK:
        case ENUM_TDC7200_COARSE_CNTR_OVF_H:
        case ENUM_TDC7200_COARSE_CNTR_OVF_L:
        case ENUM_TDC7200_CLOCK_CNTR_OVF_H:
        case ENUM_TDC7200_CLOCK_CNTR_OVF_L:
        case ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_H:
        case ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L:
            length = one_byte_data_spi_length;
            HAL_SPI_TransmitReceive(&hspi2, tx_buff_byte, rx_buff_byte, length, 0xFFFF);
            rx_buff[0] = rx_buff_byte[1];
            break;
        case ENUM_TDC7200_TIME_1:
        case ENUM_TDC7200_CLOCK_COUNT_1:
        case ENUM_TDC7200_TIME_2:
        case ENUM_TDC7200_CLOCK_COUNT_2:
        case ENUM_TDC7200_TIME_3:
        case ENUM_TDC7200_CLOCK_COUNT_3:
        case ENUM_TDC7200_TIME_4:
        case ENUM_TDC7200_CLOCK_COUNT_4:
        case ENUM_TDC7200_TIME_5:
        case ENUM_TDC7200_CLOCK_COUNT_5:
        case ENUM_TDC7200_TIME_6:
        case ENUM_TDC7200_CALIBRATION_1:
        case ENUM_TDC7200_CALIBRATION_2:
            length = three_byte_data_spi_length;
            HAL_SPI_TransmitReceive(&hspi2, tx_buff_3_byte, rx_buff_3_byte, length, 0xFFFF);
            rx_buff[0] = rx_buff_3_byte[3];
            rx_buff[1] = rx_buff_3_byte[2];
            rx_buff[2] = rx_buff_3_byte[1];
            break;
        default:
            return;
	}

	HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);

}
