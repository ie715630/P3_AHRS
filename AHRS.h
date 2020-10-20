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

#define acc_range 2	// rango de 2g
#define acc_max_val 32768	// INT16 con signo
#define gyr_range 2000	// rango de 2000°/s
#define gyr_max_val 32768	// INT16 con signo

#define HEADER_VAL 0xAAAAAAAA
#define AHRS_IMU_SAMPLE_TIME 20			// 10 ms - f= 100 Hz
//#define EVENT_MUESTRASREADY (1 << 0)

typedef struct {
	uint32_t header;
	float x;
	float y;
	float z;
}uart_msg_t;

typedef struct{
	float x;
	float y;
	float z;
}BMI160_float_data_t;

#endif /* P3_AHRS_AHRS_H_ */
