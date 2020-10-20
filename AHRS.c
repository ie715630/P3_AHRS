/*
 * AHRS.c
 *
 *  Created on: Oct 19, 2020
 *      Author: sergio_mndz
 */

#include "AHRS.h"
#include "math.h"

bmi160_float_data_t float_conversion_gyr(bmi160_raw_data_t data)
{
	bmi160_float_data_t gyr_converted_data;
	const float conversion_factor_gyr = (float) gyr_range / (float) gyr_max_val;
	gyr_converted_data.x = (float) data.x * conversion_factor_gyr;
	gyr_converted_data.y = (float) data.y * conversion_factor_gyr;
	gyr_converted_data.z = (float) data.z * conversion_factor_gyr;
	return gyr_converted_data;
}

bmi160_float_data_t float_conversion_acc(bmi160_raw_data_t data)
{
	bmi160_float_data_t acc_converted_data;
	const float conversion_factor_acc = (float) acc_range / (float) acc_max_val;
	acc_converted_data.x = (float) data.x * conversion_factor_acc;
	acc_converted_data.y = (float) data.y * conversion_factor_acc;
	acc_converted_data.z = (float) data.z * conversion_factor_acc;
	return acc_converted_data;
}

