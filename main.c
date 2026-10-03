/**
 * @file    main.c
 * @brief   H题车载平衡滚球控制工程
 * @note    本调试分支只初始化管球所需硬件；循线硬件在恢复生产任务时启用。
 */

#include "ti_msp_dl_config.h"   /* SysConfig 自动生成的硬件配置 */
#include "led/led.h"             /* LED 指示灯 (PA22) */
#include "led/key.h"             /* 用户按键 (PB21, 内部上拉, 按下低电平) */
#include "task/app_tasks.h"      /* FreeRTOS 任务创建与启动 */


static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init();
    key_init();
}

int main(void)
{
    prvSetupHardware();
    app_tasks_start();
    while (1) {}
}
