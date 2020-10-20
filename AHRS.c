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

comm_msg_t  data_conversion_nav(void)
{
	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyr_data;
	bmi160_float_data_t acc_flt_data;
	bmi160_float_data_t gyr_flt_data;
	MahonyAHRSEuler_t local_euler;
	comm_msg_t data_message;

	acc_data = bmi160_i2c_read_acc();
	gyr_data = bmi160_i2c_read_gyr();

	acc_flt_data = float_conversion_acc(acc_data);
	gyr_flt_data = float_conversion_gyr(gyr_data);

	local_euler = MahonyAHRSupdateIMU(gyr_flt_data.x, gyr_flt_data.y, gyr_flt_data.z,
									acc_flt_data.x, acc_flt_data.y, acc_flt_data.z);
	data_message.header = HEADER_VAL;
	data_message.x = local_euler.roll;
	data_message.y = local_euler.pitch;
	data_message.z = local_euler.yaw;
	return data_message;
}
