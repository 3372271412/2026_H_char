/**
 * @file    delay.c
 * @brief   MSPM0 延时与时间戳工具 (FreeRTOS 感知)
 */
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"

int delay_ms(uint32_t num_ms)
{
    if (num_ms == 0U) {
        return 0;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(num_ms));
    } else {
        while (num_ms--) {
            delay_cycles(CPUCLK_FREQ / 1000U);
        }
    }
    return 0;
}

int get_time_ms(uint32_t *count)
{
    if (count == NULL) {
        return 1;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        *count = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    } else {
        *count = 0U;
    }
    return 0;
}