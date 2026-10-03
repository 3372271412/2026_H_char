/**
 * @file    key.c
 * @brief   按键检测模块 (PB21, 内部上拉, 按下低电平)
 * @note
 *   ── 消抖算法 ──
 *   软件消抖: 连续两次读到相同有效电平才确认状态变化
 *   原理: key_read_user() 每次被调用时读取 GPIO 电平,
 *         若本次与上次相同 → 更新去抖后状态
 *         若本次与上次不同 → 保持上次去抖后状态 (拒绝抖动)
 *
 *   ── 时序要求 ──
 *   消抖窗口: 2 次调用间隔
 *   调用周期: 建议 10ms (speed_gear_task 中的 vTaskDelay(10ms))
 *   按键响应延迟: ≤ 20ms (最多 2 个周期)
 *   机械按键抖动: 通常 < 5ms, 2 次采样即可滤除
 *
 *   ── 使用示例 ──
 *   @code
 *     key_init();
 *     bool last = false;
 *     while (1) {
 *         bool now = key_read_user();
 *         if (now && !last) {  // 上升沿 (按下瞬间)
 *             // 处理按键事件
 *         }
 *         last = now;
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 *   @endcode
 */
#include "key.h"
#include "ti_msp_dl_config.h"

/* ── 引脚宏 (由 SysConfig 生成) ── */
#define KEY_PORT        GPIO_KEY_USER_PORT
#define KEY_PIN         GPIO_KEY_USER_PIN_KEY_USER_PIN
#define SELECT_KEY_PORT GPIO_KEY_TASK_PORT
#define SELECT_KEY_PIN  GPIO_KEY_TASK_PIN_KEY_TASK_PIN

/* ── 两个按键各自独立消抖 ── */
static bool last_raw  = false;
static bool debounced = false;
static bool select_last_raw = false;
static bool select_debounced = false;

/**
 * @brief  按键初始化
 * @note   清零消抖状态机
 *         PB21 引脚配置 (内部上拉/输入) 已由 SysConfig 完成
 */
void key_init(void)
{
    last_raw  = false;
    debounced = false;
    select_last_raw = false;
    select_debounced = false;
}

/**
 * @brief  读取按键状态 (含软件消抖)
 * @return true = 按下 (低电平), false = 释放 (高电平)
 *
 * @note   消抖逻辑:
 *         1. 读取当前 GPIO 电平 (低电平 = 按下 = true)
 *         2. 若本次与上次相同 → 认为稳定, 更新 debounced
 *         3. 若本次与上次不同 → 保持 debounced 不变 (拒绝抖动)
 *         4. 保存本次原始值供下次比较
 *
 *         消抖窗口 = 2 次调用间隔, 噪声脉冲宽度 < 调用间隔时被滤除
 */
bool key_read_user(void)
{
    /* 读取原始电平: 低电平(DL_GPIO_readPins==0) = 按下 = true */
    bool raw = (DL_GPIO_readPins(KEY_PORT, KEY_PIN) == 0);

    /* 消抖: 连续两次相同才更新 */
    if (raw == last_raw) {
        debounced = raw;
    }

    /* 保存本次原始值供下次比较 */
    last_raw = raw;

    return debounced;
}

bool key_read_task_select(void)
{
    /* PA18：内部下拉，高电平按下。 */
    bool raw = (DL_GPIO_readPins(SELECT_KEY_PORT, SELECT_KEY_PIN) != 0U);
    if (raw == select_last_raw) {
        select_debounced = raw;
    }
    select_last_raw = raw;
    return select_debounced;
}
