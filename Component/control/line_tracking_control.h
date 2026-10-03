/**
 * @file line_tracking_control.h
 * @brief 灰度循迹纯控制算法接口。
 *
 * @details
 * 输入一帧已归一化灰度数据，按固定顺序完成：
 * @code
 * 强度判线 -> 位置/PD -> 丢线与重捕 -> 弯道基础速度
 *          -> base±turn -> 最终轮速只限加速整形
 * @endcode
 * 输出可直接交给速度内环。不依赖FreeRTOS、编码器、电机驱动或速度
 * 服务。各任务通过 params 指针读取自己在 task_params.c 中的参数块。
 */
#ifndef LINE_TRACKING_CONTROL_H
#define LINE_TRACKING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "gray_sensor/gray_sensor.h"
#include "pid/pid.h"
#include "config/task_params.h"

typedef enum {
    LINE_TRACKING_PROFILE_TASK2_LAP = 0,
    LINE_TRACKING_PROFILE_TASK4_COMBINED,
    LINE_TRACKING_PROFILE_TASK5_LAP_BALL,
    LINE_TRACKING_PROFILE_TASK6_SPECIFIED_BALL
} line_tracking_profile_t;

/**
 * @brief 纯控制器跨10ms周期保存的状态。
 * @note 这些成员是算法历史，不是调参入口；调参入口为 params 指向的
 *       g_taskN_params.line，默认值集中在 config/task_params.c。
 */
typedef struct {
    pid_pos_t steering_pid;          /**< 转向位置式PD实例 */
    int32_t requested_base_rpm;      /**< START请求的直线基础速度 */
    int32_t current_base_rpm;        /**< 弯道调速后的当前基础速度 */
    int32_t filtered_position;       /**< 位置低通内部状态 */
    int32_t last_position;           /**< 上一有效黑线位置 */
    int32_t last_turn;               /**< 上周期turn，供丢线和重捕复用 */
    int32_t last_left_target_rpm;    /**< 轮速加速整形的左轮历史 */
    int32_t last_right_target_rpm;   /**< 轮速加速整形的右轮历史 */
    uint16_t lost_ticks;             /**< 连续丢线10ms周期数 */
    uint16_t corner_recovery_ticks;  /**< 弯道速度保持剩余周期 */
    uint8_t reacquire_blend_ticks;   /**< 重捕turn平滑剩余10ms周期 */
    uint8_t release_stable_ticks;    /**< 连续满足方向解锁条件的周期数 */
    uint8_t wheel_accel_interval_ticks; /**< 轮速缓升分频计数 */
    uint16_t run_ticks;          /**< 本次运行10ms周期计数 */
    int8_t search_direction;         /**< 最近明确黑线方向：-1/+1 */
    int8_t corner_turn_sign;         /**< 丢线后锁存的turn符号：-1/+1 */
    bool kp_high_active;             /**< 当前是否使用大偏差Kp */
    bool steering_line_was_lost;     /**< 上周期丢线，触发turn重捕平滑 */
    bool motion_line_was_lost;       /**< 丢线后重捕，触发弯速保持 */
    bool corner_direction_locked;    /**< true=尚未确认完成本次过弯 */
    bool line_ever_detected;         /**< 本次运行是否曾检测到黑线 */
    bool checkpoint_active;          /**< true=当前处于全黑检查点 */
    uint8_t checkpoint_count;        /**< 已遇到全黑检查点的次数 */
    uint8_t checkpoint_stop_count;   /**< 0=只报告检查点，不自动停车 */
    volatile line_control_params_t *params; /**< 当前菜单任务独立参数块 */
} line_tracking_control_t;

/** 一帧纯控制输出，由service直接转交速度服务并发布诊断快照。 */
typedef struct {
    int32_t position;                /**< 黑线位置，左负右正，约±7×1024 */
    int32_t steering_error;          /**< PD误差，等于滤波位置的相反数 */
    int32_t base_rpm;                /**< 弯道调速后的当前基础速度 */
    int32_t turn_rpm;                /**< 平滑后的转向差速 */
    int32_t requested_turn_rpm;      /**< 重捕blend/通用slew之前的请求差速 */
    int32_t left_target_rpm;         /**< 加速整形后的左轮目标 */
    int32_t right_target_rpm;        /**< 加速整形后的右轮目标 */
    uint32_t line_strength;          /**< 八路黑色强度总和 */
    uint16_t line_peak_strength;     /**< 单路最大黑色强度 */
    uint8_t strong_channel_count;    /**< 明确黑色通道数 */
    bool reacquire_blending;         /**< true=turn正在执行重捕平滑 */
    bool checkpoint_detected;        /**< true=本帧检测到全黑检查点 */
    uint8_t checkpoint_count;        /**< 已遇到全黑检查点的次数 */
    bool sensor_valid;               /**< true=灰度帧与标定有效 */
    bool line_detected;              /**< true=强黑通道为1~4路；0或5~8路为丢线 */
    bool stop_required;              /**< true=控制器请求service故障停车 */
} line_tracking_output_t;

/**
 * @brief 初始化控制器并装入菜单Task4的默认参数指针。
 * @param ctx 控制器状态，非空
 * @note 系统初始化时调用一次；每次运行使用start，不要重复init。
 */
void line_tracking_control_init(line_tracking_control_t *ctx);

/** 在停车状态下装入任务专属循线参数并复位控制器。 */
void line_tracking_control_select_profile(line_tracking_control_t *ctx,
                                          line_tracking_profile_t profile);

/**
 * @brief 清空本次运行的PD、丢线、弯速和轮速整形状态。
 * @param ctx 控制器状态，非空
 * @note 停车时调用；不会重新创建对象或访问任何硬件。
 */
void line_tracking_control_reset(line_tracking_control_t *ctx);

/**
 * @brief 开始一次纯控制运行。
 * @param ctx       控制器状态
 * @param base_rpm  START请求的基础速度，必须为正且已由service检查上限
 * @note 内部先reset，再按base_rpm设置动态turn限幅和初始基础速度。
 */
void line_tracking_control_start(line_tracking_control_t *ctx,
                                 int32_t base_rpm);

/** 启动循线并指定自动停车的检查点次数；0表示只报告检查点。 */
void line_tracking_control_start_checkpoint(line_tracking_control_t *ctx,
                                            int32_t base_rpm,
                                            uint8_t checkpoint_stop_count);

/**
 * @brief 用一帧灰度更新完整循迹控制输出。
 * @param ctx     已start的控制器状态
 * @param sensor  本周期八路归一化灰度帧
 * @param out     完整控制结果；函数进入后先清零
 * @note 每10ms只调用一次。stop_required为true时左右轮输出保持为0，
 *       service必须执行停车而不能复用上一周期目标。
 */
void line_tracking_control_update(line_tracking_control_t *ctx,
                                  const gray_sensor_data_t *sensor,
                                  line_tracking_output_t *out);

/**
 * @brief 无状态分析一帧灰度，仅供停止态UI/串口预览。
 * @param sensor 本周期八路归一化灰度帧
 * @param out    仅sensor_valid/line_detected/位置/强度字段有效
 * @note 不推进PD、丢线、方向锁存或轮速整形状态，不会产生轮速目标。
 */
void line_tracking_control_preview(const line_tracking_control_t *ctx,
                                   const gray_sensor_data_t *sensor,
                                   line_tracking_output_t *out);

#endif /* LINE_TRACKING_CONTROL_H */
