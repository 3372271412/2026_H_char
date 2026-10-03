/**
 * @file    speed_loop_core.h
 * @brief   速度环纯算法核心：滤波、斜坡、PID、低速前馈、PWM 计算。
 *
 * @details 本模块不依赖 FreeRTOS、编码器驱动或 TB6612，仅做纯算法运算，
 *          便于移植到其它硬件适配层或做单元测试。硬件适配由 speed_service
 *          负责：读取编码器增量传入，取输出 PWM 写入电机驱动。
 *
 *          节拍约定：10ms 采样，50ms 控制周期（5 次累计一个窗口）。
 */
#ifndef SPEED_LOOP_CORE_H
#define SPEED_LOOP_CORE_H

#include "pid/pid.h"
#include "config/task_params.h"
#include <stdbool.h>
#include <stdint.h>

/** 速度环控制周期 50ms。 */
#define SPEED_LOOP_CORE_CONTROL_PERIOD_MS 50U
/** 速度环采样周期 10ms。 */
#define SPEED_LOOP_CORE_SAMPLE_PERIOD_MS  10U

/**
 * @brief 速度环核心运行时状态。
 * @note  结构体成员由核心内部维护，外部只通过 getter 读取输出。
 */
typedef struct {
    pid_inc_t left_pid;          /**< 左轮增量式 PID 实例 */
    pid_inc_t right_pid;         /**< 右轮增量式 PID 实例 */
    int32_t left_rpm10_filt;     /**< 左轮 RPM×10 一阶低通滤波值 */
    int32_t right_rpm10_filt;    /**< 右轮 RPM×10 一阶低通滤波值 */
    int32_t left_target_rpm;     /**< 左轮目标速度（外部设定） */
    int32_t right_target_rpm;    /**< 右轮目标速度（外部设定） */
    int32_t left_setpoint_rpm;   /**< 左轮斜坡后的实际设定点 */
    int32_t right_setpoint_rpm;  /**< 右轮斜坡后的实际设定点 */
    int32_t left_delta_sum;      /**< 左轮编码器增量累计（窗口内） */
    int32_t right_delta_sum;     /**< 右轮编码器增量累计（窗口内） */
    uint32_t sample_count;       /**< 当前窗口已采样次数 */
    int32_t left_rpm;            /**< 左轮实测 RPM（滤波后） */
    int32_t right_rpm;           /**< 右轮实测 RPM（滤波后） */
    int32_t left_pwm;            /**< 左轮 PWM 输出 */
    int32_t right_pwm;           /**< 右轮 PWM 输出 */
    volatile speed_control_params_t *params; /**< 当前菜单任务独立速度参数 */
} speed_loop_core_t;

/**
 * @brief 速度环核心输出快照。
 * @note  brake=true 时应调用 tb6612_brake() 而非 set_speed()。
 */
typedef struct {
    int32_t left_rpm;            /**< 左轮实测 RPM */
    int32_t right_rpm;           /**< 右轮实测 RPM */
    int32_t left_target_rpm;     /**< 左轮目标 RPM */
    int32_t right_target_rpm;    /**< 右轮目标 RPM */
    int32_t left_pwm;            /**< 左轮 PWM */
    int32_t right_pwm;           /**< 右轮 PWM */
    bool stopped;                /**< true=已完全停稳 */
    bool brake;                  /**< true=应制动（短接电机） */
} speed_loop_core_output_t;

/** 初始化核心：清零状态并指向菜单Task4的独立速度参数。 */
void speed_loop_core_init(speed_loop_core_t *ctx);

/** 清零核心并切换到指定任务速度参数块。仅允许在车轮制动时调用。 */
void speed_loop_core_configure(
    speed_loop_core_t *ctx,
    volatile speed_control_params_t *params);

/** 设置左右轮目标速度。目标均为 0 时立即停止并返回 true（调用方据此制动）。 */
bool speed_loop_core_set_target(speed_loop_core_t *ctx,
                                int32_t left_rpm,
                                int32_t right_rpm);

/** 10ms 采样入口：累计编码器增量，满 50ms 窗口执行 PID。
 *  返回 true 表示本周期完成了 PID 计算与 PWM 更新（调用方应输出电机）。 */
bool speed_loop_core_update(speed_loop_core_t *ctx,
                            int32_t left_delta,
                            int32_t right_delta,
                            uint32_t sample_period_ms,
                            uint32_t counts_per_rev,
                            bool start_ff_enable,
                            speed_loop_core_output_t *out);

/** 只读取出当前输出快照（不执行计算）。 */
void speed_loop_core_get_output(const speed_loop_core_t *ctx,
                                speed_loop_core_output_t *out);

/** 便捷查询：目标、斜坡设定点与 PWM 是否全部归零。 */
bool speed_loop_core_is_stopped(const speed_loop_core_t *ctx);

#endif /* SPEED_LOOP_CORE_H */
