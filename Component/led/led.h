/**
 * @file    led.h
 * @brief   LED 指示灯控制接口
 */
#ifndef LED_H
#define LED_H

#include <stdint.h>

/** @brief 初始化 LED (GPIO 已由 SysConfig 配置) */
void led_init(void);

/** @brief 翻转 LED (1Hz 心跳闪烁) */
void led_toggle(void);

/** @brief 点亮 LED (低电平驱动) */
void led_on(void);

/** @brief 熄灭 LED (高电平) */
void led_off(void);

#endif /* LED_H */
