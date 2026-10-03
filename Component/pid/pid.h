/**
 * @file    pid.h
 * @brief   增量式 PID 控制器接口定义 (千倍整数运算, 无浮点)
 *
 * ── 设计说明 ──
 * 1. 千倍整数: 所有系数 × 1000 存储, 保留 3 位小数精度, 避免浮点运算
 * 2. 增量式: 输出 = 上次输出 + 增量 Δu, 天然抗积分饱和
 * 3. 输出限幅: 在累积值上做 clamp, 等效于积分分离
 * 4. 独立控制器: 每个控制轴 (左轮/右轮) 需要独立的 pid_inc_t 实例
 *
 * ── 使用方式 ──
 * @code
 *   pid_inc_t motor_pid;
 *   // Kp=0.08, Ki=0.05, Kd=0, 输出范围 ±50
 *   pid_inc_init(&motor_pid, 80, 50, 0, -50, 50);
 *
 *   // 每个控制周期:
 *   int32_t output = pid_inc_compute(&motor_pid, target_rpm, actual_rpm);
 *   tb6612_set_speed(output, output);
 *
 *   // 目标变更时复位:
 *   pid_inc_reset(&motor_pid);
 * @endcode
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief 增量式 PID 控制器结构体
 *
 * 成员说明:
 *   kp_milli/ki_milli/kd_milli: 系数 × 1000 (例: 80 = Kp 0.080)
 *   out_min/out_max: 输出限幅
 *   output: 当前输出值 (外部读取)
 *   output_milli: 输出 × 1000 (内部精度保持)
 *   err_1: 上一次误差 e[k-1]
 *   err_2: 上上次误差 e[k-2]
 */
typedef struct {
    int32_t kp_milli;       /**< 比例系数 × 1000 */
    int32_t ki_milli;       /**< 积分系数 × 1000 */
    int32_t kd_milli;       /**< 微分系数 × 1000 */
    int32_t out_min;        /**< 输出下限 */
    int32_t out_max;        /**< 输出上限 */
    int32_t output;         /**< 当前输出值 (限幅后) */
    int32_t output_milli;   /**< 输出 × 1000 (内部高精度) */
    int32_t err_1;          /**< 上周期误差 e[k-1] */
    int32_t err_2;          /**< 上上周期误差 e[k-2] */
} pid_inc_t;

/**
 * @brief 位置式 PID 控制器结构体
 * @note  用于 yaw/循线等位置量闭环，输入通常是“误差”，输出为转向差速量。
 */
typedef struct {
    int32_t kp_milli;          /**< 比例系数 × 1000 */
    int32_t ki_milli;          /**< 积分系数 × 1000 */
    int32_t kd_milli;          /**< 微分系数 × 1000 */
    int32_t out_min;           /**< 输出下限 */
    int32_t out_max;           /**< 输出上限 */
    int32_t output;            /**< 当前输出值 (限幅后) */
    int32_t integral_milli;    /**< 积分累计项 × 1000 */
    int32_t last_err;          /**< 上周期误差 */
    uint8_t first_run;         /**< 首次运行标记，避免 D 项突变 */
} pid_pos_t;

/**
 * @brief  初始化增量式 PID 控制器并复位状态
 * @param  pid       控制器指针
 * @param  kp_milli  比例系数 × 1000 (如 80 = 0.080)
 * @param  ki_milli  积分系数 × 1000
 * @param  kd_milli  微分系数 × 1000
 * @param  out_min   输出下限
 * @param  out_max   输出上限
 */
void pid_inc_init(pid_inc_t *pid, int32_t kp_milli, int32_t ki_milli,
                  int32_t kd_milli, int32_t out_min, int32_t out_max);

/**
 * @brief  在线调整 PID 系数 (不改变内部状态)
 * @param  pid       控制器指针
 * @param  kp_milli  新比例系数 × 1000
 * @param  ki_milli  新积分系数 × 1000
 * @param  kd_milli  新微分系数 × 1000
 */
void pid_inc_set_gain(pid_inc_t *pid, int32_t kp_milli, int32_t ki_milli,
                      int32_t kd_milli);

/**
 * @brief  在线调整输出限幅并约束当前输出
 * @param  pid      控制器指针
 * @param  out_min  新输出下限
 * @param  out_max  新输出上限
 */
void pid_inc_set_output_limit(pid_inc_t *pid, int32_t out_min, int32_t out_max);

/**
 * @brief  复位 PID 控制器 (输出清零, 清除历史误差)
 * @param  pid  控制器指针
 */
void pid_inc_reset(pid_inc_t *pid);

/**
 * @brief  执行一次 PID 计算
 * @param  pid       控制器指针
 * @param  target    目标值 (设定点)
 * @param  measured  实测值 (反馈值)
 * @return 限幅后的输出值
 */
int32_t pid_inc_compute(pid_inc_t *pid, int32_t target, int32_t measured);

/**
 * @brief  获取当前输出值 (不执行计算)
 * @param  pid  控制器指针
 * @return 当前输出值
 */
int32_t pid_inc_get_output(const pid_inc_t *pid);

/**
 * @brief  初始化位置式 PID 控制器并复位状态
 */
void pid_pos_init(pid_pos_t *pid, int32_t kp_milli, int32_t ki_milli,
                  int32_t kd_milli, int32_t out_min, int32_t out_max);

/**
 * @brief  在线调整位置式 PID 系数
 */
void pid_pos_set_gain(pid_pos_t *pid, int32_t kp_milli, int32_t ki_milli,
                      int32_t kd_milli);

/**
 * @brief  在线调整位置式 PID 输出限幅并约束当前输出
 */
void pid_pos_set_output_limit(pid_pos_t *pid, int32_t out_min, int32_t out_max);

/**
 * @brief  复位位置式 PID 控制器
 */
void pid_pos_reset(pid_pos_t *pid);

/**
 * @brief  按误差执行一次位置式 PID 计算
 * @param  pid  控制器指针
 * @param  err  当前误差，例如 yaw_error_deg10
 * @return 限幅后的输出值
 * @note   内部自动计算积分项 (integral += ki * err)，适合常规闭环。
 *         对于需要积分分离/动态限幅等扩展策略的场景，使用
 *         pid_pos_compute_with_integral() + getter/setter 自行管理积分。
 */
int32_t pid_pos_compute_error(pid_pos_t *pid, int32_t err);

/**
 * @brief  接受外部积分项，执行位置式 PID 的 P + D + 限幅 + 状态维护
 * @param  pid              控制器指针
 * @param  err              当前误差
 * @param  integral_milli   外部预计算的积分项 ×1000（调用方自行做分离/衰减/限幅）
 * @param  derr_out         输出：本周期误差变化（NULL=不需要），首周期返回 0
 * @return 限幅后的输出值
 * @note   本函数不修改 pid->integral_milli，调用方通过 getter/setter 管理积分。
 *         更新 pid->output / pid->last_err / pid->first_run。
 */
int32_t pid_pos_compute_with_integral(pid_pos_t *pid,
                                       int32_t err,
                                       int32_t integral_milli,
                                       int32_t *derr_out);

/**
 * @brief  获取/设置 积分项 (milli)，供外部策略（如 yaw 积分分离）读写
 */
int32_t pid_pos_get_integral_milli(const pid_pos_t *pid);
void    pid_pos_set_integral_milli(pid_pos_t *pid, int32_t integral_milli);

/**
 * @brief  获取 积分系数 / 输出上限，供外部策略做条件判断
 */
int32_t pid_pos_get_ki_milli(const pid_pos_t *pid);
int32_t pid_pos_get_out_max(const pid_pos_t *pid);

/** @brief 计算 derr + 处理 first_run，供策略层提前获取误差变化。
 *   @return 首周期 0，之后为 err - last_err。内部推进 first_run 状态。 */
int32_t pid_pos_get_derr_start(pid_pos_t *pid, int32_t err);

/**
 * @brief  获取位置式 PID 当前输出值
 */
int32_t pid_pos_get_output(const pid_pos_t *pid);

#endif /* PID_H */
