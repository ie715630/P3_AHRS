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

SemaphoreHandle_t i2c_initilized;

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

    xSemaphoreGive(i2c_initilized);
    for (;;)
    {
        vTaskSuspend(NULL);
    }
}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    i2c_initilized = xSemaphoreCreateBinary();

    xTaskCreate(init_modules, "init_BMI160", 110, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        __asm volatile ("nop");
    }
    return 0 ;
}
