/**
 * @file    distance_control.h
 * @brief   距离控制纯算法核心：匀速行驶 + 到点制动。
 *
 * @details 本模块不依赖 FreeRTOS、编码器驱动、TB6612，仅做纯运算。
 *          硬件与 RTOS 适配由 line_tracking_service 负责：周期性读取速度环
 *          状态快照中的累计计数，传入本模块，取输出的左右轮 RPM 下发
 *          速度环。
 *
 *          运动形态：直行（左右轮同速同向），统计左右轮平均累计计数。
 *          距离单位：编码器计数；每圈计数由当前菜单任务的
 *          g_taskN_params.speed.counts_per_rev 标定，不在本模块硬编码。
 *
 *          停车策略：匀速行驶，接近目标（target - brake_margin）时直接
 *          下发 (0,0) 由速度环制动；brake_margin 用于补偿惯性过冲，需
 *          实测标定。
 */
#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/** 距离控制状态机状态。 */
typedef enum {
    DIST_STATE_IDLE = 0,    /**< 空闲：未启动或已到达并复位 */
    DIST_STATE_DRIVING,     /**< 行驶中：匀速下发 */
    DIST_STATE_REACHED      /**< 已到达：本周期触发了停止 */
} dist_state_t;

/** 距离控制运行时状态。 */
typedef struct {
    int32_t       target_counts;    /**< 目标脉冲数（绝对值，正） */
    int32_t       cruise_rpm;       /**< 巡航速度（带符号：正=前进，负=后退） */
    int32_t       start_avg;        /**< 启动时刻的平均累计计数基准 */
    int32_t       brake_margin;     /**< 制动余量（脉冲）：提前这么多脉冲下发停止 */
    dist_state_t  state;            /**< 当前状态 */
    int32_t       travelled;        /**< 已行驶脉冲（绝对值） */
    int32_t       remaining;        /**< 剩余脉冲（绝对值，到点后为 0） */
} distance_control_t;

/** 初始化（置为 IDLE）。 */
void distance_control_init(distance_control_t *ctx);

/**
 * @brief  启动一次距离行驶。
 * @param  ctx            状态指针
 * @param  target_counts  目标脉冲数（正；负数会被取绝对值）
 * @param  cruise_rpm     巡航速度（带符号；0 视为无效，不启动）
 * @param  brake_margin   制动余量（脉冲，>=0）
 * @param  current_avg    启动时刻的左右轮平均累计计数
 * @note   若 brake_margin >= target_counts，启动后立即标记到达。
 */
void distance_control_start(distance_control_t *ctx,
                            int32_t target_counts,
                            int32_t cruise_rpm,
                            int32_t brake_margin,
                            int32_t current_avg);

/** 启动只负责进度/到达判断的距离监督，不要求巡航速度。 */
void distance_control_start_progress(distance_control_t *ctx,
                                     int32_t target_counts,
                                     int32_t brake_margin,
                                     int32_t current_avg);

/**
 * @brief  10ms 监督入口。
 * @param  ctx           状态指针
 * @param  current_avg   当前左右轮平均累计计数
 * @param  out_left_rpm  输出：左轮目标 RPM
 * @param  out_right_rpm 输出：右轮目标 RPM
 * @retval true  本周期状态发生变化（进入 REACHED），调用方据此制动
 * @retval false 状态未变（IDLE/DRIVING）
 * @note   IDLE/REACHED 状态下输出 (0,0)。DRIVING 状态输出 cruise_rpm。
 *         到达 target - brake_margin 时切到 REACHED 并输出 (0,0)。
 */
bool distance_control_update(distance_control_t *ctx,
                             int32_t current_avg,
                             int32_t *out_left_rpm,
                             int32_t *out_right_rpm);

/**
 * @brief 仅更新距离进度和到达状态，不产生左右轮速度目标。
 * @retval true 本周期刚进入 REACHED；false 未到达或原本已停止。
 * @note 供循迹定距复用，转向差速仍由循迹控制器产生。
 */
bool distance_control_update_progress(distance_control_t *ctx,
                                      int32_t current_avg);

/** 紧急中止：立即置 REACHED，后续 update 输出 (0,0)。 */
void distance_control_abort(distance_control_t *ctx);

/** 是否处于行驶中（DRIVING）。 */
bool distance_control_is_active(const distance_control_t *ctx);

/** 是否已到达（REACHED）。 */
bool distance_control_is_reached(const distance_control_t *ctx);

#endif /* DISTANCE_CONTROL_H */
