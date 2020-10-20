/*
 * AHRS.h
 *
 *  Created on: Oct 19, 2020
 *      Author: sergio_mndz
 */

#ifndef P3_AHRS_AHRS_H_
#define P3_AHRS_AHRS_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "BMI160.h"
#include "freertos_uart.h"
#include "mahony.h"

#define acc_range 2.0	// rango de 2g
#define acc_max_val 32768.0	// INT16 con signo
#define gyr_range 2000.0	// rango de 2000°/s
#define gyr_max_val 32768.0	// INT16 con signo

#define HEADER_VAL 0xAAAAAAAA

typedef struct {
    uint32_t header;
    float x;
    float y;
    float z;
}comm_msg_t;

typedef struct{
    float x;
    float y;
    float z;
}bmi160_float_data_t;

bmi160_float_data_t float_conversion_gyr(bmi160_raw_data_t data);

bmi160_float_data_t float_conversion_acc(bmi160_raw_data_t data);

comm_msg_t  data_conversion_nav(void);

#endif /* P3_AHRS_AHRS_H_ */
