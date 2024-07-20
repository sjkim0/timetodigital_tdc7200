/*
 * tdc7200.c
 *
 *  Created on: Jul 19, 2024
 *      Author: ksj10
 */



#include "tdc7200.h"
#include "spi.h"


tdc7200_t tdc7200_inst;


static void _tdc7200WriteSpi(uint8_t addr, uint8_t *tx_data);
static void _tdc7200ReadSpi(uint8_t addr, uint8_t *rx_buff);
static void _readAllReg(void);
static void _startMeasure(void);


void tdc7200Init(void)
{
	/* TODO:
	 *
	 * @ 0. Enable TDC
	 * @ 1. Read All Register
	 */

	// @ 0. Enable TDC
//    HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
//    delay(10);
//	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
//	delay(10);
//	HAL_GPIO_WritePin(OEN_GPIO_Port, OEN_Pin, GPIO_PIN_SET);
//	delay(10);
    HAL_GPIO_WritePin(OEN_GPIO_Port, OEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
    delay(100);
	// @ 1. Read All Register
    while(true)
    {
        _readAllReg();
    }

	_startMeasure();
}

void tdc7200Main(void)
{
    /*
     * TODO: READ GPIO_PIN, WRITE USER CMD
     * @ 1. READ TRIGG PIN
     * @ 2. READ INTERRUPT PIN
     */
    // * @ 1. READ TRIGG PIN
    tdc7200_inst.trigg_pin = HAL_GPIO_ReadPin(TRG_GPIO_Port, TRG_Pin);
    if(tdc7200_inst.trigg_pin == GPIO_PIN_SET)
    {
        tdc7200_inst.trigg_pin = GPIO_PIN_RESET;
    }

    // * @ 2. READ INTERRUPT PIN
    tdc7200_inst.interrupt_pin = HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin);
    if(tdc7200_inst.interrupt_pin == GPIO_PIN_SET)
    {
        tdc7200_inst.interrupt_pin = GPIO_PIN_RESET;
        _readAllReg();
    }
}

static void _tdc7200WriteSpi(uint8_t addr, uint8_t *tx_data)
{
    int length = 0;
    int one_byte_data_spi_length = 2;
    int three_byte_data_spi_length = 4;
    uint8_t tx_buff[4] = {addr | (1 << 6), };

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
            break;
        default:
            return;
    }
    for(int i = 0; i < length - 1; i ++)
    {
        tx_buff[i + 1] = tx_data[i];
    }
    HAL_SPI_Transmit(&hspi2, tx_buff, length, 0xFFFF);

    HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
}

static void _tdc7200ReadSpi(uint8_t addr, uint8_t *rx_buff)
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

static void _readAllReg(void)
{
    int start_index = ENUM_TDC7200_CONFIG_1;
    int stop_index = ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L;
    for(int i = start_index; i <= stop_index; i++)
    {
        _tdc7200ReadSpi(i, &tdc7200_inst.reg_1_byte_data[i]);
        delay(100);
    }
    start_index = ENUM_TDC7200_TIME_1 - ENUM_TDC7200_TIME_1;
    stop_index = ENUM_TDC7200_CALIBRATION_2 - ENUM_TDC7200_TIME_1;
    for(int i = start_index; i <= stop_index; i++)
    {
        _tdc7200ReadSpi(i + ENUM_TDC7200_TIME_1, tdc7200_inst.reg_3_byte_data[i]);
        delay(100);
    }
}

static void _startMeasure(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= 1 << 0;

    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}
