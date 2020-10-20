/**
 * @file    P3_AHRS.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "semphr.h"
#include "freertos_i2c.h"
#include "BMI160.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_uart.h"
#include "mahony.h"
#include "AHRS.h"

typedef union {
  float data;
  unsigned char bytes[4];
} float_in_bytes;

SemaphoreHandle_t modules_initialized;

void init_uart();
void init_modules(void *parameters);
void convert_msg_data_to_array(uint8_t * data, comm_msg_t ahrs_msg);
void AHRS(void *parameters);

int main(void)
{
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    modules_initialized= xSemaphoreCreateBinary();

    xTaskCreate(init_modules, "init_BMI160", 110, NULL, 1, NULL);
    xTaskCreate(AHRS, "AHRS", 500, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        __asm volatile ("nop");
    }
    return 0 ;
}

void init_uart()
{
    freertos_uart_config_t config;

    config.baudrate = 115200;
    config.rx_pin = 16;
    config.tx_pin = 17;
    config.pin_mux = kPORT_MuxAlt3;
    config.uart_number = freertos_uart0;
    config.port = freertos_uart_portB;
    freertos_uart_init(config);
}


void init_modules(void *parameters)
{
    bmi160_i2c_initialization();
    init_uart();

    xSemaphoreGive(modules_initialized);
    for (;;)
    {
        vTaskSuspend(NULL);
    }
}

void AHRS(void *parameters)
{
    xSemaphoreTake(modules_initialized, portMAX_DELAY);

    for (;;)
    {
        comm_msg_t ahrs_msg = data_conversion_nav();

        uint8_t data_for_uart[20];
        convert_msg_data_to_array(data_for_uart, ahrs_msg);
        freertos_uart_send(freertos_uart0, data_for_uart, 16);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void convert_msg_data_to_array(uint8_t * data, comm_msg_t ahrs_msg)
{
    float_in_bytes f_data[3];
    f_data[0].data = ahrs_msg.x;
    f_data[1].data = ahrs_msg.y;
    f_data[2].data = ahrs_msg.z;

    data[3] = data[2] = data[1] = data[0] = 0xAA;
    
    uint8_t byte_cnt = 4;

    for (uint8_t xyz = 0; xyz < 3; xyz++)
    {
        for (uint8_t current_byte = 0; current_byte < 4; current_byte++)
        {
            data[current_byte + byte_cnt] = f_data[xyz].bytes[current_byte];
        }
        byte_cnt += 4;
    }
}

