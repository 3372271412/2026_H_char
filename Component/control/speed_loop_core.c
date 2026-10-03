/**
 * @file    speed_loop_core.c
 * @brief   速度环纯算法核心实现：滤波、斜坡、PID、低速前馈、PWM 计算。
 *
 * @details 不依赖 FreeRTOS/编码器驱动/TB6612，仅做纯运算。硬件适配由
 *          speed_service 负责。节拍：10ms 采样，5 次累计一个 50ms 窗口做 PID。
 *          单位约定：RPM 为整数，内部滤波用 RPM×10 保留精度；PWM 为整数。
 */
#include "control/speed_loop_core.h"
#include "common/util.h"
#include <stddef.h>

/** TB6612公开接口的绝对PWM安全上限；任务参数只能在此范围内收紧。 */
#define SPEED_PWM_HARD_LIMIT 100

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void get_safe_pwm_limits(volatile speed_control_params_t *params,
                                int32_t *out_min,
                                int32_t *out_max)
{
    int32_t min_value = clamp_i32(params->pwm_min,
                                  -SPEED_PWM_HARD_LIMIT,
                                  SPEED_PWM_HARD_LIMIT);
    int32_t max_value = clamp_i32(params->pwm_max,
                                  -SPEED_PWM_HARD_LIMIT,
                                  SPEED_PWM_HARD_LIMIT);
    if (min_value > max_value) {
        int32_t temp = min_value;
        min_value = max_value;
        max_value = temp;
    }
    *out_min = min_value;
    *out_max = max_value;
}

/* 编码器增量转 RPM×10：rpm10 = delta*600000 / (counts_per_rev*period_ms)。
 * 600000 = 60s/min × 1000ms/s × 10；用 int64_t 防溢出。 */
static int32_t delta_to_rpm10(int32_t delta,
                              uint32_t period_ms,
                              uint32_t counts_per_rev)
{
    if (period_ms == 0U || counts_per_rev == 0U) {
        return 0;
    }
    return (int32_t)(((int64_t)delta * 600000) /
                     ((int64_t)counts_per_rev * period_ms));
}

/* 左右轮 PID 装入指定参数。 */
static void pid_init(pid_inc_t *left_pid, pid_inc_t *right_pid,
                     volatile speed_control_params_t *params)
{
    int32_t pwm_min;
    int32_t pwm_max;
    get_safe_pwm_limits(params, &pwm_min, &pwm_max);
    pid_inc_init(left_pid, params->kp_milli, params->ki_milli,
                 params->kd_milli, pwm_min, pwm_max);
    pid_inc_init(right_pid, params->kp_milli, params->ki_milli,
                 params->kd_milli, pwm_min, pwm_max);
}

/* 目标速度斜坡：步长来自当前任务 .speed.ramp_step_rpm。 */
static int32_t ramp_step(int32_t current, int32_t target, int32_t step)
{
    if (step < 1) {
        step = 1;
    }

    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }

    return current;
}

/* 低速启动前馈：克服静摩擦，仅在 enable 且设定点/误差满足条件时生效。
 * 设定点低于 FULL 设定点时前馈按比例衰减，避免小目标过冲。 */
static int32_t apply_start_feedforward(
    int32_t pwm,
    int32_t setpoint_rpm,
    int32_t measured_rpm,
    bool enable,
    volatile speed_control_params_t *params)
{
    int64_t ff_value = params->start_ff_pwm;
    if (ff_value < 0) {
        ff_value = -ff_value;
    }
    if (ff_value > SPEED_PWM_HARD_LIMIT) {
        ff_value = SPEED_PWM_HARD_LIMIT;
    }
    int32_t ff_pwm = (int32_t)ff_value;

    int32_t abs_setpoint = util_abs_i32(setpoint_rpm);
    if (!enable || ff_pwm == 0 || setpoint_rpm == 0 ||
        abs_setpoint > params->start_ff_max_setpoint_rpm ||
        abs_setpoint < params->start_ff_min_setpoint_rpm ||
        util_abs_i32(setpoint_rpm - measured_rpm) <=
            params->start_ff_error_rpm) {
        return pwm;
    }

    int32_t full_setpoint = params->start_ff_full_setpoint_rpm;
    if (full_setpoint < 1) {
        full_setpoint = 1;
    }
    if (abs_setpoint < full_setpoint) {
        int32_t min_ff_pwm = clamp_i32(params->start_ff_min_pwm,
                                       0, SPEED_PWM_HARD_LIMIT);
        int32_t ff_span = ff_pwm - min_ff_pwm;
        if (ff_span > 0) {
            ff_pwm = min_ff_pwm +
                (ff_span * abs_setpoint) / full_setpoint;
        }
    }

    int32_t sign = (setpoint_rpm > 0) ? 1 : -1;
    if (pwm != 0 && ((pwm > 0 && sign < 0) || (pwm < 0 && sign > 0))) {
        return pwm;
    }
    if (util_abs_i32(pwm) >= ff_pwm) {
        return pwm;
    }
    return (sign > 0) ? ff_pwm : -ff_pwm;
}

void speed_loop_core_configure(
    speed_loop_core_t *ctx,
    volatile speed_control_params_t *params)
{
    if (ctx == NULL || params == NULL) {
        return;
    }

    speed_loop_core_t init = {0};
    *ctx = init;
    ctx->params = params;
    pid_init(&ctx->left_pid, &ctx->right_pid, params);
}

void speed_loop_core_init(speed_loop_core_t *ctx)
{
    speed_loop_core_configure(ctx, &g_task4_params.speed);
}

bool speed_loop_core_set_target(speed_loop_core_t *ctx,
                                int32_t left_rpm,
                                int32_t right_rpm)
{
    if (ctx == NULL) {
        return false;
    }

    ctx->left_target_rpm = left_rpm;
    ctx->right_target_rpm = right_rpm;

    /* 目标为 0/0：立即清零设定点/PWM/并复位 PID，返回 true 要求调用方制动。 */
    if (left_rpm == 0 && right_rpm == 0) {
        ctx->left_setpoint_rpm = 0;
        ctx->right_setpoint_rpm = 0;
        ctx->left_pwm = 0;
        ctx->right_pwm = 0;
        pid_inc_reset(&ctx->left_pid);
        pid_inc_reset(&ctx->right_pid);
        return true;
    }

    return false;
}

void speed_loop_core_get_output(const speed_loop_core_t *ctx,
                                speed_loop_core_output_t *out)
{
    if (ctx == NULL || out == NULL) {
        return;
    }

    out->left_rpm = ctx->left_rpm;
    out->right_rpm = ctx->right_rpm;
    out->left_target_rpm = ctx->left_target_rpm;
    out->right_target_rpm = ctx->right_target_rpm;
    out->left_pwm = ctx->left_pwm;
    out->right_pwm = ctx->right_pwm;
    out->stopped = speed_loop_core_is_stopped(ctx);
    out->brake = out->stopped;
}

bool speed_loop_core_update(speed_loop_core_t *ctx,
                            int32_t left_delta,
                            int32_t right_delta,
                            uint32_t sample_period_ms,
                            uint32_t counts_per_rev,
                            bool start_ff_enable,
                            speed_loop_core_output_t *out)
{
    if (ctx == NULL) {
        return false;
    }

    /* 累计编码器增量，未满 50ms 窗口不执行 PID。 */
    ctx->left_delta_sum += left_delta;
    ctx->right_delta_sum += right_delta;
    ctx->sample_count++;

    if (ctx->sample_count <
        (SPEED_LOOP_CORE_CONTROL_PERIOD_MS / SPEED_LOOP_CORE_SAMPLE_PERIOD_MS)) {
        if (out != NULL) {
            speed_loop_core_get_output(ctx, out);
        }
        return false;
    }

    /* 窗口满：计算 RPM×10，并按当前任务参数做一阶低通。 */
    uint32_t period_ms = ctx->sample_count * sample_period_ms;
    int32_t left_rpm10 = delta_to_rpm10(ctx->left_delta_sum,
                                        period_ms,
                                        counts_per_rev);
    int32_t right_rpm10 = delta_to_rpm10(ctx->right_delta_sum,
                                         period_ms,
                                         counts_per_rev);

    int32_t filter_alpha = ctx->params->rpm_filter_alpha_milli;
    if (filter_alpha < 0) filter_alpha = 0;
    if (filter_alpha > 1000) filter_alpha = 1000;
    ctx->left_rpm10_filt += (int32_t)((int64_t)filter_alpha *
        (left_rpm10 - ctx->left_rpm10_filt) / 1000);
    ctx->right_rpm10_filt += (int32_t)((int64_t)filter_alpha *
        (right_rpm10 - ctx->right_rpm10_filt) / 1000);
    ctx->left_delta_sum = 0;
    ctx->right_delta_sum = 0;
    ctx->sample_count = 0;

    /* 目标斜坡 + PID 计算 + 低速前馈。每个控制窗口重装参数，
     * 使 Keil Watch 在线调节仅作用于当前任务。 */
    pid_inc_set_gain(&ctx->left_pid, ctx->params->kp_milli,
                     ctx->params->ki_milli, ctx->params->kd_milli);
    pid_inc_set_gain(&ctx->right_pid, ctx->params->kp_milli,
                     ctx->params->ki_milli, ctx->params->kd_milli);
    int32_t safe_pwm_min;
    int32_t safe_pwm_max;
    get_safe_pwm_limits(ctx->params, &safe_pwm_min, &safe_pwm_max);
    pid_inc_set_output_limit(&ctx->left_pid, safe_pwm_min, safe_pwm_max);
    pid_inc_set_output_limit(&ctx->right_pid, safe_pwm_min, safe_pwm_max);
    ctx->left_setpoint_rpm = ramp_step(ctx->left_setpoint_rpm,
        ctx->left_target_rpm, ctx->params->ramp_step_rpm);
    ctx->right_setpoint_rpm = ramp_step(ctx->right_setpoint_rpm,
        ctx->right_target_rpm, ctx->params->ramp_step_rpm);
    ctx->left_rpm = ctx->left_rpm10_filt / 10;
    ctx->right_rpm = ctx->right_rpm10_filt / 10;

    ctx->left_pwm = pid_inc_compute(&ctx->left_pid,
                                    ctx->left_setpoint_rpm,
                                    ctx->left_rpm);
    ctx->right_pwm = pid_inc_compute(&ctx->right_pid,
                                     ctx->right_setpoint_rpm,
                                     ctx->right_rpm);

    ctx->left_pwm = apply_start_feedforward(ctx->left_pwm,
        ctx->left_setpoint_rpm, ctx->left_rpm, start_ff_enable, ctx->params);
    ctx->right_pwm = apply_start_feedforward(ctx->right_pwm,
        ctx->right_setpoint_rpm, ctx->right_rpm, start_ff_enable, ctx->params);
    ctx->left_pwm = clamp_i32(ctx->left_pwm,
                              -SPEED_PWM_HARD_LIMIT,
                              SPEED_PWM_HARD_LIMIT);
    ctx->right_pwm = clamp_i32(ctx->right_pwm,
                               -SPEED_PWM_HARD_LIMIT,
                               SPEED_PWM_HARD_LIMIT);

    /* 目标与设定点均归零时清输出并复位 PID，确保完全停稳。 */
    if (ctx->left_target_rpm == 0 && ctx->right_target_rpm == 0 &&
        ctx->left_setpoint_rpm == 0 && ctx->right_setpoint_rpm == 0) {
        ctx->left_pwm = 0;
        ctx->right_pwm = 0;
        pid_inc_reset(&ctx->left_pid);
        pid_inc_reset(&ctx->right_pid);
    }

    if (out != NULL) {
        speed_loop_core_get_output(ctx, out);
    }
    return true;   /* 本周期完成了 PID 计算与 PWM 更新 */
}

bool speed_loop_core_is_stopped(const speed_loop_core_t *ctx)
{
    if (ctx == NULL) {
        return true;
    }
    return (ctx->left_target_rpm == 0 && ctx->right_target_rpm == 0 &&
            ctx->left_setpoint_rpm == 0 && ctx->right_setpoint_rpm == 0 &&
            ctx->left_pwm == 0 && ctx->right_pwm == 0);
}
