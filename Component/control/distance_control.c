/**
 * @file    distance_control.c
 * @brief   距离控制纯算法核心实现：匀速行驶 + 到点制动。
 *
 * @details 不依赖 FreeRTOS/编码器驱动/TB6612，仅做纯运算与状态机维护。
 *          调用方（line_tracking_service）负责：
 *            - 周期性读取速度环状态快照中的累计计数，计算平均计数传入；
 *            - 取输出的左右轮 RPM 下发速度环；
 *            - update 返回 true 时由速度环的 (0,0) 目标触发制动。
 */
#include "control/distance_control.h"
#include <limits.h>
#include <stddef.h>

void distance_control_init(distance_control_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    distance_control_t init = {0};
    init.state = DIST_STATE_IDLE;
    *ctx = init;
}

static void distance_control_start_common(distance_control_t *ctx,
                                          int32_t target_counts,
                                          int32_t cruise_rpm,
                                          int32_t brake_margin,
                                          int32_t current_avg,
                                          bool require_cruise)
{
    if (ctx == NULL) {
        return;
    }

    int64_t target64 = target_counts;
    if (target64 < 0) {
        target64 = -target64;
    }
    if (target64 > INT32_MAX) {
        target64 = INT32_MAX;
    }
    ctx->target_counts = (int32_t)target64;
    ctx->cruise_rpm = cruise_rpm;
    ctx->brake_margin = (brake_margin < 0) ? 0 : brake_margin;
    ctx->start_avg = current_avg;
    ctx->travelled = 0;
    ctx->remaining = ctx->target_counts;

    if (ctx->target_counts == 0 || (require_cruise && cruise_rpm == 0)) {
        ctx->state = DIST_STATE_IDLE;
    } else if (ctx->brake_margin >= ctx->target_counts) {
        ctx->state = DIST_STATE_REACHED;
    } else {
        ctx->state = DIST_STATE_DRIVING;
    }
}

void distance_control_start(distance_control_t *ctx,
                            int32_t target_counts,
                            int32_t cruise_rpm,
                            int32_t brake_margin,
                            int32_t current_avg)
{
    distance_control_start_common(ctx, target_counts, cruise_rpm,
                                  brake_margin, current_avg, true);
}

void distance_control_start_progress(distance_control_t *ctx,
                                     int32_t target_counts,
                                     int32_t brake_margin,
                                     int32_t current_avg)
{
    distance_control_start_common(ctx, target_counts, 0,
                                  brake_margin, current_avg, false);
}

bool distance_control_update_progress(distance_control_t *ctx,
                                      int32_t current_avg)
{
    if (ctx == NULL || ctx->state == DIST_STATE_IDLE ||
        ctx->state == DIST_STATE_REACHED) {
        return false;
    }

    /* 使用64位中间量，避免累计计数相减及 abs(INT32_MIN) 溢出。 */
    int64_t travelled64 = (int64_t)current_avg - ctx->start_avg;
    if (travelled64 < 0) {
        travelled64 = -travelled64;
    }
    if (travelled64 > INT32_MAX) {
        travelled64 = INT32_MAX;
    }
    int32_t travelled = (int32_t)travelled64;
    ctx->travelled = travelled;

    /* 触发点：target - brake_margin；margin 已保证 < target。 */
    int32_t trigger = ctx->target_counts - ctx->brake_margin;
    if (travelled >= trigger) {
        ctx->state = DIST_STATE_REACHED;
        ctx->remaining = 0;
        return true;
    }

    ctx->remaining = ctx->target_counts - travelled;
    return false;
}

bool distance_control_update(distance_control_t *ctx,
                             int32_t current_avg,
                             int32_t *out_left_rpm,
                             int32_t *out_right_rpm)
{
    if (out_left_rpm != NULL)  { *out_left_rpm = 0; }
    if (out_right_rpm != NULL) { *out_right_rpm = 0; }

    if (ctx == NULL || ctx->state == DIST_STATE_IDLE ||
        ctx->state == DIST_STATE_REACHED) {
        return false;
    }

    if (distance_control_update_progress(ctx, current_avg)) {
        return true;
    }

    /* 直线定距适配层仍输出左右同速；循迹只调用 progress 接口。 */
    if (out_left_rpm != NULL)  { *out_left_rpm = ctx->cruise_rpm; }
    if (out_right_rpm != NULL) { *out_right_rpm = ctx->cruise_rpm; }
    return false;
}

void distance_control_abort(distance_control_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state     = DIST_STATE_REACHED;
    ctx->remaining = 0;
}

bool distance_control_is_active(const distance_control_t *ctx)
{
    return (ctx != NULL && ctx->state == DIST_STATE_DRIVING);
}

bool distance_control_is_reached(const distance_control_t *ctx)
{
    return (ctx != NULL && ctx->state == DIST_STATE_REACHED);
}
