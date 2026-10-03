/**
 * @file    led.c
 * @brief   LED 指示灯控制模块 (PA22)
 * @note    LED 引脚: PA22 (已在 SysConfig GPIO_LED 中配置)
 *          LED 极性: 低电平点亮 (clear 亮, set 灭)
 *          GPIO 初始状态已由 SYSCFG_DL_init() 配置
 */
#include "led.h"
#include "ti_msp_dl_config.h"

/**
 * @brief  LED 初始化
 * @note   GPIO 已在 SYSCFG_DL_init() 中初始化, 此处无额外操作
 *         保留此函数以保持 API 一致性和扩展性
 */
void led_init(void)
{
    /* GPIO 已在 SYSCFG_DL_init() 中初始化, 无需额外配置 */
}

/**
 * @brief  翻转 LED 状态 (亮→灭 / 灭→亮)
 * @note   用于心跳指示, led_task 每 500ms 调用一次
 *         实现 1Hz 闪烁 (亮 500ms + 灭 500ms)
 */
void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN_22_PIN);
}

/**
 * @brief  点亮 LED (低电平)
 * @note   LED 低电平点亮 (硬件设计: LED 阳极为 VCC, 阴极为 PA22)
 */
void led_on(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
}

/**
 * @brief  熄灭 LED (高电平)
 */
void led_off(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN_22_PIN);
}
