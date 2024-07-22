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
static void _mode1Operate(void);
static void _mode2Operate(void);

/*
 * CONFIG 1 SET
 */
static void _startMeasure(void);
static void _measureModeOneSet(void);
static void _measureModeTwoSet(void);
static void _stopedgeRisingSet(void);
static void _stopedgeFallingSet(void);
static void _forceCalSet(void);
/*
 * CONFIG 2 SET
 */
static void _calibrate2Period(void);
static void _avgCycles(void);
static void _numStopSet(void);

/*
 * COMBINE SET
 */

static void _CalOperateSet(void);

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
    // TODO: INIT instance
    tdc7200_inst.resolution_ps = 55;  // 55ps
    tdc7200_inst.period_ns_mode_1 = 0;
    tdc7200_inst.period_ns_offset_mode_1 = 0;  // measured value
    tdc7200_inst.stop_falling_set = false;
    tdc7200_inst.mode_2 = false;
    tdc7200_inst.num_stop = ENUM_NUM_STOP_FIVE;  // two stops
    tdc7200_inst.calibrate2_period = ENUM_CALIBRATE2_2_CLOCK;
    tdc7200_inst.avg_cycle = ENUM_AVG_CYCLE_1;
    tdc7200_inst.mode_2_resolution_ns = 125; // 8 MHz oscillator
	/* TODO:
	 *
	 * @ 0. Enable TDC
	 * @ 1. Read All Register
	 * @ 2. Set MEAS_MODE
     * @ 3. Set edge
     * @ 4. Force calibration
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
    if(tdc7200_inst.mode_2 == false)
    {
        _measureModeOneSet();
    }
    else
    {
        _measureModeTwoSet();
    }
    delay(100);
    // * @ 3. Set edge
    if(tdc7200_inst.stop_falling_set == true)
    {
        _stopedgeFallingSet();
    }
    else
    {
        _stopedgeRisingSet();
    }
    delay(100);
    // * @ 4. Force calibration
    _numStopSet();
    delay(100);
    _calibrate2Period();
    delay(100);
    _avgCycles();
    delay(100);

    double ns_buff = 0;
    for(int i = 0; i < DEF_TICK_BUFF_LEN; i++)
    {
        _CalOperateSet();
        delay(10);
        _startMeasure();
        delay(10);
        _readAllReg();

        int calibrate_2_index = ENUM_TDC7200_CALIBRATION_2 - ENUM_TDC7200_TIME_1;
        uint32_t tick_buff = tdc7200_inst.reg_3_byte_data[calibrate_2_index][0];
        tick_buff |= (tdc7200_inst.reg_3_byte_data[calibrate_2_index][1] << 8);
        tick_buff |= (tdc7200_inst.reg_3_byte_data[calibrate_2_index][2] << 16);

        tdc7200_inst.tick_buff[i] = tick_buff;
    }
    for(int i = 0; i < DEF_TICK_BUFF_LEN; i++)
    {
        ns_buff += tdc7200_inst.tick_buff[i] / 1000.0F * 55.0F;
    }
    ns_buff /= DEF_TICK_BUFF_LEN;

    tdc7200_inst.calibrate_ns = ns_buff;

    delay(100);
}

void tdc7200Main(void)
{
    /*
     * TODO: READ GPIO_PIN, WRITE USER CMD
     * @ 1. READ ALL REGISTER
     * @ 2. Start Measure
     */

    // * @ 1. READ ALL REGISTER
    // tdc7200_inst.interrupt_pin = HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin);
    _readAllReg();

    if(tdc7200_inst.mode_2 == false)
    {
        _mode1Operate();
    }
    else
    {
        _mode2Operate();
    }

    _startMeasure();

    delay(1);
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

static void _mode1Operate(void)
{
    /*
     * TODO: Tick이 2group화 되어 기록된다. 이중 값이 더 작은 그룹이 real Data에 더 가깝다.
     * 1. 2개의 그룹의 평균을 계산
     * 2. 값이 작은 그룹의 데이터중 중간 데이터를 fetch한다.
     */
    uint32_t timer4_tick = tdc7200_inst.reg_3_byte_data[6][0];
    timer4_tick |= (tdc7200_inst.reg_3_byte_data[6][1] << 8);
    timer4_tick |= (tdc7200_inst.reg_3_byte_data[6][2] << 16);
    uint32_t timer5_tick = tdc7200_inst.reg_3_byte_data[8][0];
    timer5_tick |= (tdc7200_inst.reg_3_byte_data[8][1] << 8);
    timer5_tick |= (tdc7200_inst.reg_3_byte_data[8][2] << 16);
    uint32_t diff_tick = 0;

    if(timer5_tick > timer4_tick)
    {
        diff_tick = timer5_tick - timer4_tick;
    }
    else
    {
        diff_tick = timer4_tick - timer5_tick;
    }
    tdc7200_inst.tick_buff[tdc7200_inst.tick_buff_index] = diff_tick;
    tdc7200_inst.tick_buff_index += 1;
    if(tdc7200_inst.tick_buff_index == DEF_TICK_BUFF_LEN)
    {
        // bubble sort for asending tick buffer;
        bool swapped = false;
        for(int i = 0; i < DEF_TICK_BUFF_LEN - 1; i++)
        {
            swapped = false;
            for(int j = 0; j < DEF_TICK_BUFF_LEN - i - 1; j++)
            {
                if(tdc7200_inst.tick_buff[j] > tdc7200_inst.tick_buff[j + 1])
                {
                    uint32_t swap_buffer = tdc7200_inst.tick_buff[j];
                    tdc7200_inst.tick_buff[j] = tdc7200_inst.tick_buff[j + 1];
                    tdc7200_inst.tick_buff[j + 1] = swap_buffer;
                    swapped = true;
                }
            }
            if(swapped == false)
            {
                break;
            }
        }
        // get average_whole_tick;
        double average_whole_tick = 0;
        for(int i = 0; i < DEF_TICK_BUFF_LEN; i++)
        {
            average_whole_tick += tdc7200_inst.tick_buff[i];
        }
        average_whole_tick /= DEF_TICK_BUFF_LEN;
        // find lower_grop_end_index
        int lower_grop_end_index = 0;

        for(int i = DEF_TICK_BUFF_LEN - 1; i >= 0; i--)
        {
            if(tdc7200_inst.tick_buff[i] < average_whole_tick)
            {
                lower_grop_end_index = i;
                break;
            }
        }

        int finded_index = lower_grop_end_index / 2;
        uint32_t tick_finded = tdc7200_inst.tick_buff[finded_index];
        float period_finded = 0;

        int ns_unit_divide = 1000;

        period_finded = tick_finded * (float)tdc7200_inst.resolution_ps / ns_unit_divide;
        period_finded -= tdc7200_inst.period_ns_offset_mode_1;

        tdc7200_inst.period_ns_mode_1 = period_finded;
    }
    tdc7200_inst.tick_buff_index %= DEF_TICK_BUFF_LEN;
}

static void _mode2Operate(void)
{
    /*
     * TODO: CHECK COUNT BUFFER,
     *       IF COUNT IS 3 DIFF COUNT RECORDED
     *       -> tdc7200_inst.period_ns_mode_2 = -1;
     */
    int count4_index = ENUM_TDC7200_CLOCK_COUNT_4 - ENUM_TDC7200_TIME_1;
    int count5_index = ENUM_TDC7200_CLOCK_COUNT_5 - ENUM_TDC7200_TIME_1;

    uint32_t count_4 = tdc7200_inst.reg_3_byte_data[count4_index][0];
    count_4 |= (tdc7200_inst.reg_3_byte_data[count4_index][1] << 8);
    count_4 |= (tdc7200_inst.reg_3_byte_data[count4_index][2] << 16);
    uint32_t count_5 = tdc7200_inst.reg_3_byte_data[count5_index][0];
    count_5 |= (tdc7200_inst.reg_3_byte_data[count5_index][1] << 8);
    count_5 |= (tdc7200_inst.reg_3_byte_data[count5_index][2] << 16);
    uint32_t diff_count = 0;

    if(count_5 > count_4)
    {
        diff_count = count_5 - count_4;
    }
    else
    {
        diff_count = count_4 - count_5;
    }
    tdc7200_inst.tick_buff[tdc7200_inst.tick_buff_index] = diff_count;
    tdc7200_inst.tick_buff_index += 1;

    uint32_t cal_buff = 0;
    if(tdc7200_inst.tick_buff_index == DEF_TICK_BUFF_LEN)
    {
        /*
         * TODO: 최대 3개의 count(major count, error count)가 buffer에 산재한다고 가정하였다.
         * 가장 다수를 차지하는 major count, 나머지를 차지하는 error count를 파악한다
         */
        uint32_t major_count = 0;
        uint32_t sub_count_1 = 0;
        uint32_t sub_count_2 = 0;
        uint32_t count_checker_0 = tdc7200_inst.tick_buff[0];
        major_count += 1;

        int32_t count_checker_1 = -1;
        int32_t count_checker_2 = -1;
        bool count_done = true;

        for(int i = 1; i < DEF_TICK_BUFF_LEN; i++)
        {
            if(count_checker_0 == tdc7200_inst.tick_buff[i])
            {
                major_count += 1;
            }
            else
            {
                if(count_checker_1 == -1)
                {
                    count_checker_1 = tdc7200_inst.tick_buff[i];
                    sub_count_1 += 1;
                }
                else
                {
                    if(count_checker_1 == tdc7200_inst.tick_buff[i])
                    {
                        sub_count_1 += 1;
                    }
                    else
                    {
                        if(count_checker_2 == -1)
                        {
                            count_checker_2 = tdc7200_inst.tick_buff[i];
                            sub_count_2 += 1;
                        }
                        else
                        {
                            // error over 3 timer count buffer recorded
                            count_done = false;
                        }
                    }
                }
            }
        }
        if(count_done == true)
        {
            if(sub_count_1 > sub_count_2)
            {
                // sub_count 1 is selected
                if(major_count > sub_count_1)
                {
                    tdc7200_inst.period_ns_mode_2 = count_checker_0 * tdc7200_inst.mode_2_resolution_ns;
                }
                else
                {
                    tdc7200_inst.period_ns_mode_2 = count_checker_1 * tdc7200_inst.mode_2_resolution_ns;
                }
            }
            else
            {
                // sub_count 2 is selected
                if(major_count > sub_count_2)
                {
                    tdc7200_inst.period_ns_mode_2 = count_checker_0 * tdc7200_inst.mode_2_resolution_ns;
                }
                else
                {
                    tdc7200_inst.period_ns_mode_2 = count_checker_2 * tdc7200_inst.mode_2_resolution_ns;
                }
            }
        }
        else
        {
            tdc7200_inst.period_ns_mode_2 = -1;
        }
    }
    tdc7200_inst.tick_buff_index %= DEF_TICK_BUFF_LEN;
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

static void _forceCalSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (1 << ENUM_TDC7200_REG_0_FORCE_CAL_1B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
}


static void _calibrate2Period(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2] |= (tdc7200_inst.calibrate2_period << ENUM_TDC7200_REG_1_CALIBRATION2_PERIOD_2B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_2, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2]);
}

static void _avgCycles(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2] |= (tdc7200_inst.avg_cycle << ENUM_TDC7200_REG_1_AVG_CYCLES_3B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_2, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2]);
}

static void _numStopSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2] |= (tdc7200_inst.num_stop << ENUM_TDC7200_REG_1_NUM_STOP_3B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_2, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_2]);
}

static void _CalOperateSet(void)
{
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (1 << ENUM_TDC7200_REG_0_FORCE_CAL_1B);
    tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1] |= (1 << ENUM_TDC7200_REG_0_START_MEAS_1B);
    _tdc7200WriteSpi(ENUM_TDC7200_CONFIG_1, &tdc7200_inst.reg_1_byte_data[ENUM_TDC7200_CONFIG_1]);
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

