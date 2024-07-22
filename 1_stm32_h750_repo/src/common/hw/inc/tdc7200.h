/*
 * tdc7200.h
 *
 *  Created on: Jul 19, 2024
 *      Author: ksj10
 */

#ifndef COMMON_HW_INC_TDC7200_H_
#define COMMON_HW_INC_TDC7200_H_


#include "hw_def.h"

/*
 * TODO: DESCRIPT TDC
 * @ 0. TRIGGER -> measure done -> make rising pulse
 * @ 1. INTTERRUPT -> start -> GPIO set, measure done -> GPIO RESET
 * @ 2. MEAS_MODE 2 -> count for reference clock
 * @ 3. MEAS_MODE 1 -> not display count, but display timer's tick value,
 *      but not fixed.. value(diff range about 40 ~ 60 in force cal)
 *      Need to check more
 */

#define DEF_TDC7200_TIME_DATA_LEN      (3U)
#define DEF_TDC7200_CALIBRATE_DATA_LEN (3U)
#define DEF_TICK_BUFF_LEN              (30U)


enum ENUM_TDC7200_REG
{
	ENUM_TDC7200_CONFIG_1,
	ENUM_TDC7200_CONFIG_2,
	ENUM_TDC7200_INT_STATUS,
	ENUM_TDC7200_INT_MASK,
	ENUM_TDC7200_COARSE_CNTR_OVF_H,
	ENUM_TDC7200_COARSE_CNTR_OVF_L,
	ENUM_TDC7200_CLOCK_CNTR_OVF_H,
	ENUM_TDC7200_CLOCK_CNTR_OVF_L,
	ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_H,
	ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L,
	ENUM_TDC7200_TIME_1,                    // REG 3 0
	ENUM_TDC7200_CLOCK_COUNT_1,             // REG 3 1
	ENUM_TDC7200_TIME_2,                    // REG 3 2
	ENUM_TDC7200_CLOCK_COUNT_2,             // REG 3 3
	ENUM_TDC7200_TIME_3,                    // REG 3 4
	ENUM_TDC7200_CLOCK_COUNT_3,             // REG 3 5
	ENUM_TDC7200_TIME_4,                    // REG 3 6
	ENUM_TDC7200_CLOCK_COUNT_4,             // REG 3 7
	ENUM_TDC7200_TIME_5,                    // REG 3 8
	ENUM_TDC7200_CLOCK_COUNT_5,             // REG 3 9
	ENUM_TDC7200_TIME_6,                    // REG 3 10
	ENUM_TDC7200_CALIBRATION_1,             // REG 3 11
	ENUM_TDC7200_CALIBRATION_2,             // REG 3 12
	ENUM_TDC7200_REG_LENGTH
};

enum ENUM_TDC7200_REG_0
{
    ENUM_TDC7200_REG_0_START_MEAS_1B,
    ENUM_TDC7200_REG_0_MEAS_MODE_2B,
    ENUM_TDC7200_REG_0_START_EDGE = 3,
    ENUM_TDC7200_REG_0_STOP_EDGE_1B,
    ENUM_TDC7200_REG_0_TRIGG_EDGE_1B,
    ENUM_TDC7200_REG_0_PARITY_EN_1B,
    ENUM_TDC7200_REG_0_FORCE_CAL_1B
};

enum ENUM_TDC7200_REG_1
{
    ENUM_TDC7200_REG_1_NUM_STOP_3B,
    ENUM_TDC7200_REG_1_AVG_CYCLES_3B = 3,
    ENUM_TDC7200_REG_1_CALIBRATION2_PERIOD_2B = 6,
};

enum ENUM_TDC7200_REG_2
{
    ENUM_TDC7200_REG_2_NEW_MEAS_INT_1B,
    ENUM_TDC7200_REG_2_COARSE_CNTR_OVF_INT_1B,
    ENUM_TDC7200_REG_2_CLOCK_CNTR_OVF_INT_1B,
    ENUM_TDC7200_REG_2_MEAS_STARTED_FLAG_1B,
    ENUM_TDC7200_REG_2_MEAS_COMPLETE_FLAG_1B
};


enum ENUM_CALIBRATE2_PERIOD
{
    ENUM_CALIBRATE2_2_CLOCK,
    ENUM_CALIBRATE2_10_CLOCK,
    ENUM_CALIBRATE2_20_CLOCK,
    ENUM_CALIBRATE2_40_CLOCK,
};

enum ENUM_AVG_CYCLE
{
    ENUM_AVG_CYCLE_1,
    ENUM_AVG_CYCLE_2,
    ENUM_AVG_CYCLE_4,
    ENUM_AVG_CYCLE_8,
    ENUM_AVG_CYCLE_16,
    ENUM_AVG_CYCLE_32,
    ENUM_AVG_CYCLE_64,
    ENUM_AVG_CYCLE_128,
};

enum ENUM_NUM_STOP
{
    ENUM_NUM_STOP_SINGLE,
    ENUM_NUM_STOP_TWO,
    ENUM_NUM_STOP_THREE,
    ENUM_NUM_STOP_FOUR,
    ENUM_NUM_STOP_FIVE,
};

/*
 * NUM STOP
 */
typedef struct
{
	uint8_t reg_1_byte_data[ENUM_TDC7200_CLOCK_CNTR_STOP_MASK_L + 1];
	uint8_t reg_3_byte_data[ENUM_TDC7200_REG_LENGTH - ENUM_TDC7200_TIME_1][DEF_TDC7200_CALIBRATE_DATA_LEN];
	GPIO_PinState trigg_pin;
	GPIO_PinState interrupt_pin;
	bool mode_2;
	bool stop_falling_set;
	uint8_t calibrate2_period; // ENUM_CALIBRATE2_PERIOD
    uint8_t avg_cycle; // ENUM_AVG_CYCLE
	uint8_t num_stop; // ENUM_NUM_STOP
	int tick_buff_index;
	uint32_t tick_buff[DEF_TICK_BUFF_LEN]; // MODE 1 TICK BUFF
	int resolution_ps;
	float period_ns_mode_1;
	float period_ns_offset_mode_1;
    float period_ns_mode_2;
	float calibrate_ns;
	uint32_t mode_2_resolution_ns;
}tdc7200_t;


extern tdc7200_t tdc7200_inst;


void tdc7200Init(void);
void tdc7200Main(void);


#endif /* COMMON_HW_INC_TDC7200_H_ */
