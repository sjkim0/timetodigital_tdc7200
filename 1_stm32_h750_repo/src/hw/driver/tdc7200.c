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
static void _measureModeOneSet(void);
static void _measureModeTwoSet(void);
static void _stopedgeRisingSet(void);
static void _stopedgeFallingSet(void);

static void _numStopSet(void);

/*
 * Check
 * Interrupt status check and clear
 */
static void _clearNewMeasInt(void);
static void _clearCoarseCntrOvfInt(void);
static void _clearClockCntrOvfInt(void);
static void _clearMeasStartedFlag(void);
static void _clearMeasCompleteFlag(void);

uint32_t ok_count = 0;
uint32_t err_count = 0;
uint32_t err_buff[10000] = {0, };

void tdc7200Init(void)
{
	/* TODO:
	 *
	 * @ 0. Enable TDC
	 * @ 1. Read All Register
	 * @ 2. Set MEAS_MODE
     * @ 3. Set edge
     * @ 4. Set number of stop
     * @ 5. Start measure
	 */

	// @ 0. Enable TDC
    HAL_GPIO_WritePin(OEN_GPIO_Port, OEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);
    delay(100);
	// @ 1. Read All Register
    _readAllReg();

    //  test code
    // @ 2. Set MEAS_MODE
    tdc7200_inst.mode_2 = true;
    if(tdc7200_inst.mode_2 == false)
    {
        _measureModeOneSet();
    }
    else
    {
        _measureModeTwoSet();
    }
    // * @ 3. Set edge
    tdc7200_inst.stop_falling_set = false;
    if(tdc7200_inst.stop_falling_set == true)
    {
        _stopedgeFallingSet();
    }
    else
    {
        _stopedgeRisingSet();
    }

    // * @ 4. Set number of stop
    tdc7200_inst.num_stop = 1;  // two stops
    _numStopSet();

    delay(10);
    // @ 5. Start measure
    _startMeasure();
}

void tdc7200Main(void)
{
    /*
     * TODO: READ GPIO_PIN, WRITE USER CMD
     * @ 1. READ ALL REGISTER
     * @ 2. Clear Interrupt status register
     */

    // * @ 1. READ ALL REGISTER
    // tdc7200_inst.interrupt_pin = HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin);
    _readAllReg();
    // * @ 2. Clear Interrupt status register
//    tdc7200_inst.interrupt_pin = HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin);
//    if(tdc7200_inst.interrupt_pin == GPIO_PIN_SET)
//    {
//        _clearNewMeasInt();
//        _clearCoarseCntrOvfInt();
//        _clearClockCntrOvfInt();
//        _clearMeasStartedFlag();
//        _clearMeasCompleteFlag();
//        _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
//        _startMeasure();
//    }
    _startMeasure();
    delay(100);
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
    }
    start_index = ENUM_TDC7200_TIME_1 - ENUM_TDC7200_TIME_1;
    stop_index = ENUM_TDC7200_CALIBRATION_2 - ENUM_TDC7200_TIME_1;
    for(int i = start_index; i <= stop_index; i++)
    {
        _tdc7200ReadSpi(i + ENUM_TDC7200_TIME_1, tdc7200_inst.reg_3_byte_data[i]);
    }
}

static void _startMeasure(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (1 << ENUM_TDC7200_REG_0_START_MEAS_1B);

    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}

static void _measureModeOneSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] &= ~(0b01 << ENUM_TDC7200_REG_0_MEAS_MODE_2B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}

static void _measureModeTwoSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (0b01 << ENUM_TDC7200_REG_0_MEAS_MODE_2B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}

static void _stopedgeRisingSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] &= ~(1 << ENUM_TDC7200_REG_0_STOP_EDGE_1B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}

static void _stopedgeFallingSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (1 << ENUM_TDC7200_REG_0_STOP_EDGE_1B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}

static void _numStopSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2] |= (tdc7200_inst.num_stop << ENUM_TDC7200_REG_1_NUM_STOP_3B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_2, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2]);
}

static void _clearNewMeasInt(void)
{
    uint8_t *p_reg_data = &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS];

    if(((*p_reg_data) & (1 << ENUM_TDC7200_REG_2_NEW_MEAS_INT_1B)) != 0)
    {
        tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS] &= ~(1 << ENUM_TDC7200_REG_2_NEW_MEAS_INT_1B);
    }
}

static void _clearCoarseCntrOvfInt(void)
{
    uint8_t *p_reg_data = &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS];

    if(((*p_reg_data) & (1 << ENUM_TDC7200_REG_2_COARSE_CNTR_OVF_INT_1B)) != 0)
    {
        tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS] &= ~(1 << ENUM_TDC7200_REG_2_COARSE_CNTR_OVF_INT_1B);
    }
}

static void _clearClockCntrOvfInt(void)
{
    uint8_t *p_reg_data = &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS];

    if(((*p_reg_data) & (1 << ENUM_TDC7200_REG_2_CLOCK_CNTR_OVF_INT_1B)) != 0)
    {
        tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS] &= ~(1 << ENUM_TDC7200_REG_2_CLOCK_CNTR_OVF_INT_1B);
    }
}

static void _clearMeasStartedFlag(void)
{
    uint8_t *p_reg_data = &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS];

    if(((*p_reg_data) & (1 << ENUM_TDC7200_REG_2_MEAS_STARTED_FLAG_1B)) != 0)
    {
        tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS] &= ~(1 << ENUM_TDC7200_REG_2_MEAS_STARTED_FLAG_1B);
    }
}

static void _clearMeasCompleteFlag(void)
{
    uint8_t *p_reg_data = &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS];

    if(((*p_reg_data) & (1 << ENUM_TDC7200_REG_2_MEAS_COMPLETE_FLAG_1B)) != 0)
    {
        tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_INT_STATUS] &= ~(1 << ENUM_TDC7200_REG_2_MEAS_COMPLETE_FLAG_1B);
    }
}

