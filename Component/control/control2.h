/**
 * @file    control2.h
 * @brief   H题六任务循线、管球与组合运动控制接口。
 *
 * @details task2_lap_* 是菜单Task2的一圈循线状态机；ball_control_*
 * 是菜单Task3~6共用的管球算法内核。任务参数统一声明在
 * config/task_params.h，默认值统一定义在 config/task_params.c。
 */
#ifndef CONTROL2_H
#define CONTROL2_H

#include <stdint.h>
#include <stdbool.h>
#include "config/task_params.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  菜单 Task2：单纯巡线一圈（H题第二问，16分）
 *
 *  赛道: AB/CD=1.5m直道, BC/DA=0.5m半径半圆弧, 周长≈6.14m
 *  要求: 按键启动 → 顺时针巡线一圈 → 停到A点, ≤20s, 停车偏差≤2cm
 *  A点有5cm垂直启停线, 过线时8路灰度全黑
 *
 *  状态流: IDLE → TRACKING → BRAKING → COMPLETE
 *  启动后跳过初始启停线 (grace period), 然后监测终点启停线
 * ═══════════════════════════════════════════════════════════════════════════ */

/** 菜单Task2一圈循线状态机状态。 */
typedef enum {
    TASK2_LAP_IDLE = 0,      /**< 未启动 */
    TASK2_LAP_TRACKING,      /**< 巡线中, 监测启停线 */
    TASK2_LAP_BRAKING,       /**< 制动中, 等待停稳 */
    TASK2_LAP_COMPLETE,      /**< 完成 */
    TASK2_LAP_FAULT          /**< 循线或速度服务故障 */
} task2_lap_state_t;

/** 菜单Task2一圈循线运行时状态快照。 */
typedef struct {
    task2_lap_state_t state;       /**< 当前状态 */
    uint32_t total_time_ms;    /**< 一圈总耗时 (ms) */
    uint8_t  digital_mask;     /**< 当前灰度掩码 */
    uint8_t  strong_channel;   /**< 强黑通道数 */
    bool     lap_detected;     /**< 是否检测到启停线 */
    bool     checkpoint_armed; /**< 已稳定驶离起点横线 */
    uint8_t  fault;            /**< line_tracking_fault_t 数值 */
} task2_lap_status_t;

/** 启动官方任务2巡线一圈。 */
bool task2_lap_start(void);

/** 停止菜单Task2一圈循线，状态归IDLE。 */
void task2_lap_stop(void);

/**
 * @brief  菜单Task2一圈循线周期入口（10ms）。
 * @retval true  一圈状态机接管了本周期控制。
 * @retval false 状态机处于IDLE/COMPLETE/FAULT，可执行其他控制。
 */
bool task2_lap_step(uint32_t dt_ms);

/** 菜单Task2是否正在执行（TRACKING或BRAKING）。 */
bool task2_lap_is_active(void);

/** 获取菜单Task2一圈循线状态快照。 */
bool task2_lap_get_status(task2_lap_status_t *out);

/** 获取状态名字符串 */
const char *task2_lap_state_name(task2_lap_state_t s);

/* ═══════════════════════════════════════════════════════════════════════════
 *  管球控制内核：供菜单Task3～Task6复用（Ball & Beam）
 *
 *  串级控制: 外环位置PID(钢珠位置→期望倾角) + 内环电机角度伺服
 *  状态流: IDLE → HOMING → TRACKING ⇄ LOST
 *
 *  依赖: Emm_V5(步进电机), maixcam(视觉), pid(位置式PID)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** 管球控制内核状态。 */
typedef enum {
    BALL_CONTROL_IDLE = 0,     /**< 未启动 */
    BALL_CONTROL_HOMING,       /**< 电机初始化, 管道回水平 */
    BALL_CONTROL_TRACKING,     /**< 正常闭环控制 */
    BALL_CONTROL_LOST          /**< 钢珠丢失, 管道回水平, 等待重现 */
} ball_control_state_t;

/** 管球控制内核运行时状态快照（OLED/调试用）。 */
typedef struct {
    ball_control_state_t state;        /**< 当前状态 */
    int32_t  target_x;          /**< 目标钢珠 X 坐标 (像素) */
    int32_t  ball_x;            /**< 当前钢珠 X 坐标 (像素) */
    int32_t  error;             /**< 位置误差 = target_x - ball_x (像素) */
    int32_t  motor_target;      /**< 电机绝对目标位置 (脉冲) */
    int32_t  desired_vel;       /**< 外环期望球速 (px/s) */
    int32_t  ball_vel;          /**< 滤波后球速 (px/s) */
    int32_t  velocity_error;    /**< 速度误差 (px/s) */
    uint32_t frame_age_ms;      /**< 最新视觉帧年龄 (ms) */
    int32_t  integral_milli;    /**< 兼容旧显示字段 */
    int32_t  derr;              /**< 兼容旧显示字段 */
    bool     reached;           /**< 钢珠是否到达目标并稳定 */
    bool     ball_valid;        /**< 相机是否检测到钢珠 */
} ball_control_status_t;

/* 所有在线参数见 config/task_params.h 中的 g_hardware_calibration 和
 * g_task1_params~g_task6_params；管球接口不导出跨任务共享调参变量。 */

/** 仅使能步进电机并将管道调平，默认使用菜单Task3参数。 */
void ball_control_prepare_level(void);

/** 按菜单任务号选择独立管球参数后调平；仅接受3/4/5/6。 */
void ball_control_prepare_level_for_menu(uint8_t menu_task);

/** 使用菜单Task3参数启动管球控制内核。 */
void ball_control_start(void);

/** 停止管球控制，管道回水平，状态归IDLE。 */
void ball_control_stop(void);

/** 紧急停止并锁存当前电机位置，不执行回水平动作。 */
void ball_control_emergency_stop(void);

/** 设置钢珠目标位置 (相机像素 X 坐标) */
void ball_control_set_target(int32_t target_x);

/**
 * @brief  管球控制内核周期入口（10ms）。
 * @retval true   本周期接管了控制 (非 IDLE)
 * @retval false  IDLE 状态, 调用方应自行执行其他控制
 */
bool ball_control_step(uint32_t dt_ms);

/** 获取管球控制内核当前状态。 */
ball_control_state_t ball_control_get_state(void);

/** 管球控制内核是否正在执行（非IDLE）。 */
bool ball_control_is_active(void);

/** 钢珠是否到达目标位置并稳定 */
bool ball_control_is_reached(void);

/** 获取状态名字符串 (OLED 显示用) */
const char *ball_control_state_name(ball_control_state_t s);

/** 获取运行时状态快照 (OLED / 调试用) */
bool ball_control_get_status(ball_control_status_t *out);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task3: 第三问 — O→+5cm→-5cm（调用ball_control_*并切换目标）
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    TASK3_IDLE = 0,
    TASK3_GO_POS5,    /**< 正往 +5cm 运动 */
    TASK3_HOLD_POS5,  /**< 到达 +5cm, 停留等待 */
    TASK3_GO_NEG5,    /**< 正往 -5cm 运动 */
    TASK3_COMPLETE    /**< 完成, 停在 -5cm */
} task3_state_t;

typedef struct {
    task3_state_t state;
    uint32_t elapsed_ms;       /**< 总耗时 */
    int32_t  target_x;         /**< 当前目标 */
    bool     reached;          /**< 当前段是否到达 */
    uint32_t hold_ms;          /**< +5cm 停留计时 (HOLD_POS5 状态用) */
    uint32_t stable_ms;        /**< 到达稳定计时 */
    int32_t  ball_x;           /**< 当前球位置 X (像素) */
    int32_t  error;            /**< 当前误差 = target - ball (像素) */
} task3_status_t;

/** 启动 Task3 O→+5cm→-5cm */
void task3_start(void);
void task3_stop(void);
bool task3_step(uint32_t dt_ms);
bool task3_is_active(void);
bool task3_get_status(task3_status_t *out);
const char *task3_state_name(task3_state_t s);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task4: 第四问 — A到B定距循线 + 钢珠中心平衡
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    TASK4_IDLE = 0,
    TASK4_TRACKING,
    TASK4_BRAKING,
    TASK4_COMPLETE,
    TASK4_FAULT,
    TASK4_PREPARE,
    TASK4_READY,
    TASK4_HOLDING
} task4_state_t;

typedef enum {
    TASK4_FAULT_NONE = 0,
    TASK4_FAULT_START,
    TASK4_FAULT_LINE,
    TASK4_FAULT_BALL_LOST,
    TASK4_FAULT_TIMEOUT
} task4_fault_t;

typedef struct {
    task4_state_t state;
    task4_fault_t fault;
    uint32_t elapsed_ms;
    int32_t target_counts;
    int32_t travelled_counts;
    int32_t remaining_counts;
    int32_t line_position;
    int32_t turn_rpm;
    int32_t left_rpm;
    int32_t right_rpm;
    int32_t ball_x;
    int32_t ball_error;
    int32_t max_abs_ball_error;
    int32_t ball_accel_ff;
    uint32_t ball_violation_ms;
    uint32_t ball_invalid_ms;
    uint8_t line_fault;
    bool ball_valid;
    bool reached;
} task4_status_t;

bool task4_prepare(void);
bool task4_start_motion(void);
bool task4_is_ready(void);
/* 兼容旧调用：等同于 task4_prepare()。 */
bool task4_start(void);
void task4_stop(void);
void task4_emergency_stop(void);
bool task4_step(uint32_t dt_ms);
bool task4_is_active(void);
bool task4_get_status(task4_status_t *out);
const char *task4_state_name(task4_state_t state);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task5: 完整一圈循线 + 钢珠中心平衡
 *
 *  离开起点A后，第一次重新稳定检测到横向启停线即停车；编码器不参与
 *  终点判断，只保留卡死保护和遥测。钢珠在行驶、制动和停车确认期间
 *  持续保持中心。
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    TASK5_IDLE = 0,
    TASK5_TRACKING,
    TASK5_BRAKING,
    TASK5_COMPLETE,
    TASK5_FAULT,
    TASK5_PREPARE,
    TASK5_HOLDING
} task5_state_t;

typedef enum {
    TASK5_FAULT_NONE = 0,
    TASK5_FAULT_START,
    TASK5_FAULT_LINE,
    TASK5_FAULT_BALL_LOST,
    TASK5_FAULT_TIMEOUT
} task5_fault_t;

typedef enum {
    TASK5_LAP_MODE_CENTER = 5,
    TASK5_LAP_MODE_SPECIFIED = 6
} task5_lap_mode_t;

typedef struct {
    task5_state_t state;
    task5_fault_t fault;
    task5_lap_mode_t mode;
    uint32_t elapsed_ms;
    int32_t ball_target_x;
    int32_t ball_x;
    int32_t ball_error;
    int32_t ball_velocity;
    int32_t ball_motor_output;
    int32_t max_abs_ball_error;
    int32_t ball_accel_ff;
    int32_t travelled_counts;
    int32_t lap_estimate_counts;
    int32_t approach_rpm;
    int32_t line_position;
    int32_t turn_rpm;
    int32_t left_rpm;
    int32_t right_rpm;
    uint32_t ball_violation_ms;
    uint32_t ball_invalid_ms;
    uint8_t digital_mask;
    uint8_t strong_channel;
    uint8_t checkpoint_count;
    uint8_t line_fault;
    bool checkpoint_detected;
    bool checkpoint_armed;
    bool ball_valid;
    bool reached;
} task5_status_t;

bool task5_start(void);
bool task6_start_auto(void);
void task5_stop(void);
void task5_emergency_stop(void);
bool task5_step(uint32_t dt_ms);
bool task5_is_active(void);
bool task5_get_status(task5_status_t *out);
const char *task5_state_name(task5_state_t state);

#endif /* CONTROL2_H */