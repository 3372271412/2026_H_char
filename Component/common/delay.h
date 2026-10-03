/**
 * @file    delay.h
 * @brief   MSPM0 延时与时间戳接口 (FreeRTOS 感知, 通用)
 */
#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

int delay_ms(uint32_t num_ms);
int get_time_ms(uint32_t *count);

#endif /* _DELAY_H_ */