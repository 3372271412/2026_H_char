/**
 * @file line_tracking_control.c
 * @brief 八路灰度循迹纯控制算法。
 *
 * @details
 * 本文件不依赖FreeRTOS、编码器、电机或速度服务。一次update完成：
 * 灰度判线、转向PD、丢线重捕、弯道基础速度、左右轮合成和轮速整形。
 * 各菜单任务参数分别位于 config/task_params.c 的 .line 参数块；
 * line_tracking_config.h 只保留传感器物理权重与位置尺度。
 */
#include "control/line_tracking_control.h"
#include "control/line_tracking_config.h"
#include "common/util.h"
#include <stddef.h>
#include <string.h>

/* ========================================================================== */
/* 灰度帧分析：无状态，可同时供运行控制和停止态预览使用                       */
/* ========================================================================== */

/** 八路物理顺序必须与AD0~AD2扫描顺序一致，数值来自统一配置。 */
static const int8_t g_line_weights[GRAY_SENSOR_CHANNEL_COUNT] =
    LINE_TRACKING_POSITION_WEIGHTS;

/**
 * @brief 从归一化灰度帧提取黑色强度、有效线标志和加权位置。
 * @param sensor 输入灰度帧；要求ADC成功且八路标定均有效
 * @param out    输出统计字段；调用者必须提前清零
 * @return true=传感器帧结构有效；false=ADC或标定掩码无效
 *
 * @note strength=4095-normalized，黑色越深数值越大。逐路强黑数量为
 * 1~4才认为看到了正常线；0路视为全白，5~8路视为过宽黑区/阴影。
 */
static bool analyze_sensor(const line_tracking_control_t *ctx,
                           const gray_sensor_data_t *sensor,
                           line_tracking_output_t *out)
{
    int64_t weighted_sum = 0;
    uint32_t strength_sum = 0U;

    if (ctx == NULL || ctx->params == NULL || sensor == NULL ||
        !sensor->adc_ok || sensor->valid_mask != 0xFFU) {
        return false;
    }
    out->sensor_valid = true;

    for (uint8_t channel = 0U;
         channel < GRAY_SENSOR_CHANNEL_COUNT;
         channel++) {
        uint32_t strength = GRAY_SENSOR_ADC_FULL_SCALE -
                            sensor->normalized[channel];
        strength_sum += strength;
        if (strength > out->line_peak_strength) {
            out->line_peak_strength = (uint16_t)strength;
        }
        if (strength >= GRAY_SENSOR_LINE_STRONG_MIN) {
            out->strong_channel_count++;
        }
        weighted_sum += (int64_t)strength * g_line_weights[channel] *
                        LINE_TRACKING_POSITION_SCALE;
    }
    out->line_strength = strength_sum;

    /* 0路强黑为全白，5路强黑为过宽黑区/阴影，均按丢线处理。
     * 6路以上强黑为全黑检查点（如停止线），设checkpoint_detected。 */
    uint8_t line_max = ctx->params->line_strong_channel_max;
    uint8_t checkpoint_min = ctx->params->checkpoint_channel_min;
    if (line_max < 1U) line_max = 1U;
    if (line_max > GRAY_SENSOR_CHANNEL_COUNT) {
        line_max = GRAY_SENSOR_CHANNEL_COUNT;
    }
    if (checkpoint_min < 1U) checkpoint_min = 1U;
    if (checkpoint_min > GRAY_SENSOR_CHANNEL_COUNT) {
        checkpoint_min = GRAY_SENSOR_CHANNEL_COUNT;
    }
    out->line_detected = out->strong_channel_count > 0U &&
        out->strong_channel_count <= line_max;
    out->checkpoint_detected = out->strong_channel_count >= checkpoint_min;
    if (out->line_detected) {
        out->position = (int32_t)(weighted_sum / strength_sum);
    }
    return true;
}

/* ========================================================================== */
/* 生命周期：init只调用一次，start/reset对应每次运行的开始和结束             */
/* ========================================================================== */

void line_tracking_control_select_profile(line_tracking_control_t *ctx,
                                          line_tracking_profile_t profile)
{
    if (ctx == NULL) {
        return;
    }

    /* 只切换“指针”，不复制参数。这样 Keil Watch 修改当前任务参数后，
     * 下一个10ms周期立即读取新值；其他任务的参数块完全不受影响。 */
    if (profile == LINE_TRACKING_PROFILE_TASK2_LAP) {
        ctx->params = &g_task2_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK4_COMBINED) {
        ctx->params = &g_task4_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK5_LAP_BALL) {
        ctx->params = &g_task5_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK6_SPECIFIED_BALL) {
        ctx->params = &g_task6_params.lap.line;
    } else {
        return;
    }

    int32_t initial_limit = ctx->params->turn_limit_initial_rpm;
    if (initial_limit < 1) {
        initial_limit = 1;
    }
    pid_pos_init(&ctx->steering_pid,
                 ctx->params->kp_high_milli,
                 ctx->params->ki_milli,
                 ctx->params->kd_milli,
                 -initial_limit,
                 initial_limit);
    line_tracking_control_reset(ctx);
}

void line_tracking_control_init(line_tracking_control_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    line_tracking_control_select_profile(
        ctx, LINE_TRACKING_PROFILE_TASK4_COMBINED);
}

void line_tracking_control_reset(line_tracking_control_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->requested_base_rpm = 0;
    ctx->current_base_rpm = 0;
    ctx->filtered_position = 0;
    ctx->last_position = 0;
    ctx->last_turn = 0;
    ctx->last_left_target_rpm = 0;
    ctx->last_right_target_rpm = 0;
    ctx->lost_ticks = 0U;
    ctx->corner_recovery_ticks = 0U;
    ctx->reacquire_blend_ticks = 0U;
    ctx->release_stable_ticks = 0U;
    ctx->wheel_accel_interval_ticks = 0U;
    ctx->run_ticks = 0U;
    ctx->search_direction = 0;
    ctx->corner_turn_sign = 0;
    ctx->kp_high_active = false;
    ctx->steering_line_was_lost = false;
    ctx->motion_line_was_lost = false;
    ctx->corner_direction_locked = false;
    ctx->line_ever_detected = false;
    ctx->checkpoint_active = false;
    ctx->checkpoint_count = 0;
    ctx->checkpoint_stop_count = ctx->params->checkpoint_stop_count;
    pid_pos_reset(&ctx->steering_pid);
}

void line_tracking_control_start_checkpoint(line_tracking_control_t *ctx,
                                            int32_t base_rpm,
                                            uint8_t checkpoint_stop_count)
{
    if (ctx == NULL || base_rpm <= 0) {
        return;
    }

    line_tracking_control_reset(ctx);
    ctx->requested_base_rpm = base_rpm;
    ctx->current_base_rpm = base_rpm;
    ctx->checkpoint_stop_count = checkpoint_stop_count;

    int32_t denominator = ctx->params->turn_limit_denominator;
    if (denominator <= 0) {
        denominator = 1;
    }
    int32_t limit =
        (base_rpm * ctx->params->turn_limit_numerator) / denominator;
    if (limit < 1) {
        limit = 1;
    }
    pid_pos_set_output_limit(&ctx->steering_pid, -limit, limit);
}

void line_tracking_control_start(line_tracking_control_t *ctx,
                                 int32_t base_rpm)
{
    line_tracking_control_start_checkpoint(
        ctx, base_rpm, ctx->params->checkpoint_stop_count);
}

/* ========================================================================== */
/* 转向控制：有效线PD、丢线方向锁存和重捕平滑                               */
/* ========================================================================== */

/** 按启动基础速度百分比计算丢线最小turn；+99实现正整数向上取整。 */
static int32_t lost_search_min_turn(const line_tracking_control_t *ctx)
{
    int32_t percent = ctx->params->lost_search_turn_percent;
    if (percent < 1) {
        percent = 1;
    }
    int32_t min_turn =
        (ctx->requested_base_rpm * percent + 99) / 100;
    return (min_turn < 1) ? 1 : min_turn;
}

/** 将turn向请求值靠近，单周期变化不超过max_step。 */
static int32_t approach_turn(int32_t current,
                             int32_t requested,
                             int32_t max_step)
{
    int32_t delta = requested - current;
    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return requested;
}

/**
 * @brief 应用通用turn slew：增大和回零/反向可使用不同步长。
 * @note 对应配置为0时直接透传；当前实车参数关闭此层，只在重捕窗口
 * 使用独立的approach_turn，避免多层滤波叠加相位滞后。
 */
static int32_t apply_turn_slew(const line_tracking_control_t *ctx,
                               int32_t requested)
{
    int32_t current = ctx->last_turn;
    int32_t current_abs = (current < 0) ? -current : current;
    int32_t requested_abs = (requested < 0) ? -requested : requested;
    bool same_direction = current == 0 || requested == 0 ||
                          ((current < 0) == (requested < 0));
    bool magnitude_increasing = same_direction &&
                                requested_abs > current_abs;
    int32_t slew = magnitude_increasing ?
        ctx->params->turn_slew_up_rpm :
        ctx->params->turn_slew_down_rpm;

    if (slew <= 0) {
        return requested;
    }
    return approach_turn(current, requested, slew);
}

/**
 * @brief 处理有效黑线帧。
 * @details 依次更新位置滤波、出弯方向解锁、PD请求和重捕turn平滑。
 * out->position/analyze统计字段已由analyze_sensor填写。
 */
static void update_detected_line(line_tracking_control_t *ctx,
                                 line_tracking_output_t *out)
{
    int32_t position = out->position;
    int32_t abs_position = (position < 0) ? -position : position;

    ctx->lost_ticks = 0U;
    ctx->last_position = position;
    ctx->line_ever_detected = true;

    /* 位置低通只作用于配置范围内的小偏差；0表示完全关闭。 */
    int32_t filter_cutoff = ctx->params->position_filter_cutoff;
    int32_t old_weight = ctx->params->position_filter_old_weight;
    if (old_weight < 0) {
        old_weight = 0;
    }
    if (filter_cutoff > 0 && abs_position < filter_cutoff) {
        ctx->filtered_position =
            (ctx->filtered_position * old_weight + position) /
            (old_weight + 1);
    } else {
        ctx->filtered_position = position;
    }
    int32_t control_position = ctx->filtered_position;

    /* 连续50ms居中或越过中心才解除丢线方向锁存。 */
    if (ctx->corner_direction_locked) {
        bool crossed_center = position != 0 &&
            ((position < 0) == (ctx->corner_turn_sign < 0));
        uint16_t release_ticks = ctx->params->direction_release_ticks;
        if (release_ticks == 0U) {
            release_ticks = 1U;
        }
        if (abs_position <= ctx->params->direction_release_position ||
            crossed_center) {
            if (ctx->release_stable_ticks < release_ticks) {
                ctx->release_stable_ticks++;
            }
        } else {
            ctx->release_stable_ticks = 0U;
        }
        if (ctx->release_stable_ticks >= release_ticks) {
            ctx->corner_direction_locked = false;
            ctx->corner_turn_sign = 0;
            ctx->release_stable_ticks = 0U;
        }
    }

    /* 锁存解除后才允许更新下一次丢线的搜索方向；中心小抖动忽略。 */
    if (!ctx->corner_direction_locked) {
        if (position < -ctx->params->search_direction_threshold) {
            ctx->search_direction = -1;
        } else if (position > ctx->params->search_direction_threshold) {
            ctx->search_direction = 1;
        }
    }

    /* 位置左负右正，差速turn采用相反误差，使车辆朝黑线方向转。 */
    out->steering_error = -control_position;
    int32_t abs_error = (out->steering_error < 0) ?
                        -out->steering_error : out->steering_error;
    if (ctx->params->kp_switch_threshold <= 0) {
        ctx->kp_high_active = true;
    } else if (ctx->kp_high_active) {
        if (abs_error < ctx->params->kp_release_threshold) {
            ctx->kp_high_active = false;
        }
    } else if (abs_error >= ctx->params->kp_switch_threshold) {
        ctx->kp_high_active = true;
    }

    /* switch=0时恒用high；每帧重新装入参数以支持Watch在线调节。 */
    int32_t selected_kp = ctx->kp_high_active ?
        ctx->params->kp_high_milli : ctx->params->kp_low_milli;
    pid_pos_set_gain(&ctx->steering_pid,
                     selected_kp,
                     ctx->params->ki_milli,
                     ctx->params->kd_milli);
    int32_t requested_turn =
        pid_pos_compute_error(&ctx->steering_pid, out->steering_error);

    if (ctx->params->deadband > 0 &&
        control_position < ctx->params->deadband &&
        control_position > -ctx->params->deadband) {
        requested_turn = 0;
    }
    /* 弯中短暂重捕可能把线投影到传感器另一侧。方向尚未稳定释放时，
     * 反向PD只允许原turn逐步回零，不能立刻反打。 */
    if (ctx->corner_direction_locked && requested_turn != 0 &&
        ((requested_turn < 0) != (ctx->corner_turn_sign < 0))) {
        requested_turn = 0;
    }
    out->requested_turn_rpm = requested_turn;

    if (ctx->steering_line_was_lost) {
        ctx->reacquire_blend_ticks =
            ctx->params->reacquire_blend_ticks;
        ctx->steering_line_was_lost = false;
    }
    if (ctx->reacquire_blend_ticks > 0U) {
        int32_t blend_step = ctx->params->reacquire_blend_step_rpm;
        if (blend_step < 1) {
            blend_step = 1;
        }
        int32_t blended_turn = approach_turn(
            ctx->last_turn, requested_turn, blend_step);
        if (blended_turn != requested_turn) {
            requested_turn = blended_turn;
            out->reacquire_blending = true;
        }
        ctx->reacquire_blend_ticks--;
    }

    out->turn_rpm = apply_turn_slew(ctx, requested_turn);
    ctx->last_turn = out->turn_rpm;
}

/**
 * @brief 处理丢线帧并维持本次弯道的搜索方向。
 * @details 首次丢线锁存last_turn符号；若当时turn为0，则由最后明确的
 * 黑线位置推导方向，仍无方向时默认向右搜索。超过搜索周期后只设置
 * stop_required，由service决定故障类型并执行ESTOP。
 */
static void update_lost_line(line_tracking_control_t *ctx,
                             line_tracking_output_t *out)
{
    ctx->lost_ticks++;
    ctx->steering_line_was_lost = true;
    ctx->reacquire_blend_ticks = 0U;
    ctx->release_stable_ticks = 0U;
    out->position = ctx->last_position;
    out->steering_error = -ctx->last_position;

    if (!ctx->corner_direction_locked) {
        if (ctx->last_turn > 0) {
            ctx->corner_turn_sign = 1;
        } else if (ctx->last_turn < 0) {
            ctx->corner_turn_sign = -1;
        } else {
            /* 居中后突然丢线时last_turn可能为0：优先沿最后黑线侧搜索；
             * 从未形成明确侧向信息时约定默认向右，保证行为确定。 */
            if (ctx->search_direction == 0) {
                ctx->search_direction = 1;
            }
            ctx->corner_turn_sign =
                (ctx->search_direction < 0) ? 1 : -1;
        }
        ctx->corner_direction_locked = true;
    }

    out->turn_rpm = ctx->last_turn;
    out->requested_turn_rpm = out->turn_rpm;
    /* 丢线期间没有可信位置误差，清除PD历史，避免重捕时D项引用旧帧。 */
    pid_pos_reset(&ctx->steering_pid);

    /* 第一帧就没有检测到线时禁止盲目启动；运行中丢线则允许限时搜索。 */
    if (ctx->lost_ticks > ctx->params->lost_search_ticks ||
        !ctx->line_ever_detected) {
        out->stop_required = true;
        return;
    }

    int32_t min_search_turn = lost_search_min_turn(ctx);
    int32_t abs_turn = (out->turn_rpm < 0) ?
                       -out->turn_rpm : out->turn_rpm;
    bool direction_changed = out->turn_rpm != 0 &&
        ((out->turn_rpm < 0) != (ctx->corner_turn_sign < 0));
    if (abs_turn < min_search_turn || direction_changed) {
        out->turn_rpm = ctx->corner_turn_sign * min_search_turn;
        out->requested_turn_rpm = out->turn_rpm;
        ctx->last_turn = out->turn_rpm;
    }
}

/* ========================================================================== */
/* 纵向速度策略：弯速规划、base±turn合成和最终轮速整形                      */
/* ========================================================================== */

/** 按比例和绝对上限计算深弯基础速度，并保证至少为1RPM。 */
static int32_t corner_base_target(const line_tracking_control_t *ctx,
                                  int32_t requested_base)
{
    int32_t corner_base =
        (requested_base * ctx->params->corner_speed_percent) / 100;
    if (corner_base > ctx->params->corner_speed_max_rpm) {
        corner_base = ctx->params->corner_speed_max_rpm;
    }
    return (corner_base < 1) ? 1 : corner_base;
}

/**
 * @brief 根据横向位置规划本周期目标基础速度。
 * @return 直线区返回requested_base，深弯/丢线返回corner_base，中间区域
 * 按|position|线性插值；本函数只规划目标，不限制变化率。
 */
static int32_t tracking_base_target(const line_tracking_control_t *ctx,
                                    int32_t requested_base,
                                    bool line_detected,
                                    int32_t position)
{
    int32_t corner_base = corner_base_target(ctx, requested_base);
    if (!line_detected) {
        return corner_base;
    }

    int32_t abs_position = (position < 0) ? -position : position;
    int32_t full_position = ctx->params->speed_full_position;
    int32_t corner_position = ctx->params->speed_corner_position;
    if (corner_position <= full_position) {
        corner_position = full_position + 1;
    }
    if (abs_position <= full_position) {
        return requested_base;
    }
    if (abs_position >= corner_position) {
        return corner_base;
    }

    int32_t span = corner_position - full_position;
    int32_t reduction = (requested_base - corner_base) *
        (abs_position - full_position) / span;
    return requested_base - reduction;
}

/**
 * @brief 以不对称斜坡让当前基础速度靠近目标。
 * @details 出弯加速使用固定小步长；进弯减速按START速度同比放大，确保
 * 例如150RPM不会仍按50RPM的减速能力处理。减速快、加速慢可降低冲线风险。
 */
static int32_t approach_base(const line_tracking_control_t *ctx,
                             int32_t current,
                             int32_t target,
                             int32_t requested_base)
{
    if (current < target) {
        int32_t accel_step = ctx->params->base_accel_rpm_per_tick;
        if (accel_step < 1) {
            accel_step = 1;
        }
        int32_t next = current + accel_step;
        return (next > target) ? target : next;
    }
    if (current > target) {
        int32_t decel_ref = ctx->params->base_decel_rpm_at_ref;
        int32_t reference_rpm = ctx->params->base_decel_ref_rpm;
        if (decel_ref < 1) {
            decel_ref = 1;
        }
        if (reference_rpm < 1) {
            reference_rpm = 1;
        }
        int32_t decel_step =
            (requested_base * decel_ref + reference_rpm - 1) /
            reference_rpm;
        if (decel_step < decel_ref) {
            decel_step = decel_ref;
        }
        int32_t next = current - decel_step;
        return (next < target) ? target : next;
    }
    return current;
}

/**
 * @brief 限制单轮目标绝对值增大的速度，不限制减速和制动。
 * @details 目标需要换向时先立即回到0，下周期再从0向反方向加速；这样
 * 避免直接跨零反打，同时不拖慢转弯内轮减速。
 */
static int32_t limit_wheel_accel(const line_tracking_control_t *ctx,
                                 int32_t current,
                                 int32_t target,
                                 bool allow_accel)
{
    if (current != 0 && target != 0 &&
        ((current < 0) != (target < 0))) {
        return 0;
    }

    int32_t current_abs = (current < 0) ? -current : current;
    int32_t target_abs = (target < 0) ? -target : target;
    if (target_abs <= current_abs) {
        return target;
    }

    if (!allow_accel) {
        return current;
    }

    int32_t step = ctx->params->wheel_accel_rpm_per_tick;
    if (step < 1) {
        step = 1;
    }
    int32_t next_abs = current_abs + step;
    if (next_abs > target_abs) {
        next_abs = target_abs;
    }
    return (target < 0) ? -next_abs : next_abs;
}

/**
 * @brief 将横向turn转换成最终双轮目标。
 * @details 顺序不能交换：先维护弯速保持状态，再平滑base，然后计算
 * left=base-turn/right=base+turn，最后分别执行只限加速整形。
 */
static void update_motion(line_tracking_control_t *ctx,
                          line_tracking_output_t *out)
{
    int32_t abs_position = (out->position < 0) ?
                           -out->position : out->position;

    ctx->run_ticks++;

    /* 启动低速保护：前 STARTUP_TICKS 周期线性提升基础速度上限。 */
    int32_t effective_base = ctx->requested_base_rpm;
    uint16_t startup_ticks = ctx->params->startup_ticks;
    int32_t startup_base_rpm = ctx->params->startup_base_rpm;
    if (startup_ticks > 0U && ctx->run_ticks <= startup_ticks) {
        int32_t startup_max = startup_base_rpm +
            (int32_t)(ctx->requested_base_rpm - startup_base_rpm) *
            (int32_t)ctx->run_ticks / (int32_t)startup_ticks;
        if (effective_base > startup_max) {
            effective_base = startup_max;
        }
    }

    /* 丢线期间保持计时器满值而不递减；重新捕线或仍处于深弯时重新
     * 装载保持时间，防止传感器刚碰到线便立刻恢复测试直线速度。 */
    if (!out->line_detected) {
        ctx->motion_line_was_lost = true;
        ctx->corner_recovery_ticks =
            ctx->params->corner_recovery_ticks;
    } else if (ctx->motion_line_was_lost ||
               abs_position >= ctx->params->speed_corner_position) {
        ctx->motion_line_was_lost = false;
        ctx->corner_recovery_ticks =
            ctx->params->corner_recovery_ticks;
    }

    int32_t target_base = tracking_base_target(
        ctx, effective_base, out->line_detected, out->position);
    /* 只有持续看见有效线才消耗保持时间；再次丢线会在上面重新装满。 */
    if (ctx->corner_recovery_ticks > 0U) {
        target_base = corner_base_target(ctx, effective_base);
        if (out->line_detected) {
            ctx->corner_recovery_ticks--;
        }
    }
    ctx->current_base_rpm = approach_base(
        ctx, ctx->current_base_rpm, target_base,
        ctx->requested_base_rpm);
    out->base_rpm = ctx->current_base_rpm;

    int32_t raw_left = util_clamp_i64_to_i32(
        (int64_t)out->base_rpm - out->turn_rpm);
    int32_t raw_right = util_clamp_i64_to_i32(
        (int64_t)out->base_rpm + out->turn_rpm);
    ctx->wheel_accel_interval_ticks++;
    uint16_t accel_interval = ctx->params->wheel_accel_interval_ticks;
    if (accel_interval == 0U) {
        accel_interval = 1U;
    }
    bool allow_accel = ctx->wheel_accel_interval_ticks >= accel_interval;
    if (allow_accel) {
        ctx->wheel_accel_interval_ticks = 0U;
    }
    ctx->last_left_target_rpm = limit_wheel_accel(
        ctx, ctx->last_left_target_rpm, raw_left, allow_accel);
    ctx->last_right_target_rpm = limit_wheel_accel(
        ctx, ctx->last_right_target_rpm, raw_right, allow_accel);
    out->left_target_rpm = ctx->last_left_target_rpm;
    out->right_target_rpm = ctx->last_right_target_rpm;
}

/* ========================================================================== */
/* 对外入口：运行态update推进状态，preview严格保持无状态                    */
/* ========================================================================== */

void line_tracking_control_update(line_tracking_control_t *ctx,
                                  const gray_sensor_data_t *sensor,
                                  line_tracking_output_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    if (ctx == NULL || !analyze_sensor(ctx, sensor, out)) {
        out->stop_required = true;
        return;
    }

    /* 全黑检查点优先：直行通过，累加次数，达到上限后停车。 */
    if (out->checkpoint_detected) {
        if (!ctx->checkpoint_active) {
            ctx->checkpoint_active = true;
            ctx->checkpoint_count++;
        }
        out->turn_rpm = 0;
        out->requested_turn_rpm = 0;
        out->line_detected = true;
        out->position = 0;
        update_motion(ctx, out);
        out->line_detected = false;  /* 阻止后续lost_line逻辑 */
        ctx->last_position = 0;
        ctx->last_turn = 0;
        if (ctx->checkpoint_stop_count > 0U &&
            ctx->checkpoint_count >= ctx->checkpoint_stop_count) {
            out->stop_required = true;
        }
        out->checkpoint_count = ctx->checkpoint_count;
        return;
    }
    ctx->checkpoint_active = false;

    /* 转向阶段可能产生stop_required；只有安全时才继续生成轮速目标。 */
    if (out->line_detected) {
        update_detected_line(ctx, out);
    } else {
        update_lost_line(ctx, out);
    }
    if (!out->stop_required) {
        update_motion(ctx, out);
    }
}

void line_tracking_control_preview(const line_tracking_control_t *ctx,
                                   const gray_sensor_data_t *sensor,
                                   line_tracking_output_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    (void)analyze_sensor(ctx, sensor, out);
}
