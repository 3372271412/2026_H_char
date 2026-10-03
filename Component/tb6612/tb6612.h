/**
 * @file    tb6612.h
 * @brief   TB6612 双路直流电机驱动接口
 *
 * ── 速度范围 ──
 * speed = -100 ~ +100
 * 正数 = 前进 (CW), 负数 = 后退 (CCW), 0 = 滑行停止
 *
 * ── 制动模式 ──
 * tb6612_stop():  滑行停止 (Hi-Z), 无制动, 靠摩擦力减速
 * tb6612_brake(): 短接制动, 快速减速, 可能过热
 *
 * ── 使用示例 ──
 * @code
 *   tb6612_init();
 *   tb6612_set_speed(50, 50);   // 两轮全速前进
 *   tb6612_set_speed(-50, 50);  // 原地左转 (差速)
 *   tb6612_stop();              // 滑行停止
 * @endcode
 */
#ifndef TB6612_H
#define TB6612_H

#include <stdint.h>

/* 0：正速度前进；1：正速度后退，并同步反转闭环速度坐标。 */
#define TB6612_REVERSE_DIRECTION  0

/**
 * @brief  初始化 TB6612 驱动
 * @note   设置方向脚为停止, 启动 PWM 计数器
 *         应在 SYSCFG_DL_init() 之后调用
 */
void tb6612_init(void);

/**
 * @brief  设置左电机速度
 * @param  speed  -100 ~ +100
 */
void tb6612_set_left_speed(int16_t speed);

/**
 * @brief  设置右电机速度
 * @param  speed  -100 ~ +100
 */
void tb6612_set_right_speed(int16_t speed);

/**
 * @brief  同时设置左右电机速度
 * @param  left   左电机速度 -100 ~ +100
 * @param  right  右电机速度 -100 ~ +100
 */
void tb6612_set_speed(int16_t left, int16_t right);

/**
 * @brief  滑行停止 (Hi-Z)
 */
void tb6612_stop(void);

/**
 * @brief  短接制动 (快速刹车)
 */
void tb6612_brake(void);

#endif /* TB6612_H */
