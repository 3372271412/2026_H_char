/**
 * @file    speed_service.c
 * @brief   速度环适配层实现：FreeRTOS 队列 + 编码器读取 + TB6612 输出。
 *
 * @details 纯算法在 speed_loop_core 中，本文件负责 RTOS 适配与硬件 I/O：
 *          - 速度目标/状态队列模块私有；
 *          - 10ms 读编码器增量，满 50ms 由核心做 PID；
 *          - 核心 update 返回 true 时输出 PWM 到 TB6612；
 *          - 目标为 0/0 时核心立即停止并要求制动。
 */
#include "service/speed_service.h"
#include "control/speed_loop_core.h"
#include "encoder/encoder.h"
#include "tb6612/tb6612.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stddef.h>

/** 速度目标消息：左右轮目标 RPM + yaw 环专用低速前馈开关。 */
typedef struct {
    int32_t left_rpm;
    int32_t right_rpm;
    bool low_speed_ff_enable;
} speed_target_msg_t;

static QueueHandle_t g_speed_target_queue = NULL;  /* 目标队列（长度 1，覆盖写） */
static QueueHandle_t g_speed_state_queue = NULL;   /* 状态队列（长度 1，覆盖写） */
static speed_loop_core_t g_speed_core;             /* 速度环纯算法核心 */
static bool g_low_speed_ff_enable = false;         /* 当前是否启用低速前馈 */
static volatile bool g_emergency_stop = false;     /* 故障停车锁存 */

/* 从速度环核心取当前输出并发布状态快照到队列。
 * left_count/right_count 为编码器累计脉冲，由 step_10ms 传入；
 * 这样上层（距离控制）只读快照即可获得累计计数，无需直接访问编码器，
 * 避免与速度环的 encoder_get_data() 产生竞态。 */
static void speed_service_publish_snapshot(int32_t left_count,
                                           int32_t right_count)
{
    speed_loop_core_output_t output = {0};
    speed_loop_core_get_output(&g_speed_core, &output);

    speed_service_state_t state = {0};
    state.left_rpm = output.left_rpm;
    state.right_rpm = output.right_rpm;
    state.left_target_rpm = output.left_target_rpm;
    state.right_target_rpm = output.right_target_rpm;
    state.stopped = output.stopped;
    state.left_count = left_count;
    state.right_count = right_count;
    (void)xQueueOverwrite(g_speed_state_queue, &state);
}

bool speed_service_select_profile(speed_service_profile_t profile)
{
    volatile speed_control_params_t *params;

    taskENTER_CRITICAL();
    bool stopped = g_emergency_stop;
    taskEXIT_CRITICAL();
    if (!stopped) {
        return false;
    }

    if (profile == SPEED_PROFILE_TASK2_LAP) {
        params = &g_task2_params.speed;
    } else if (profile == SPEED_PROFILE_TASK4_COMBINED) {
        params = &g_task4_params.speed;
    } else if (profile == SPEED_PROFILE_TASK5_LAP_BALL) {
        params = &g_task5_params.speed;
    } else if (profile == SPEED_PROFILE_TASK6_SPECIFIED_BALL) {
        params = &g_task6_params.lap.speed;
    } else {
        return false;
    }
    if (params->counts_per_rev == 0U) {
        return false;
    }

    taskENTER_CRITICAL();
    speed_loop_core_configure(&g_speed_core, params);
    g_low_speed_ff_enable = false;
    taskEXIT_CRITICAL();
    encoder_reset();
    if (g_speed_target_queue != NULL) {
        (void)xQueueReset(g_speed_target_queue);
    }
    speed_service_publish_snapshot(0, 0);
    return true;
}

bool speed_service_init(void)
{
    /* 创建目标/状态队列（长度 1，覆盖写，只保留最新值）。 */
    if (g_speed_target_queue == NULL) {
        g_speed_target_queue = xQueueCreate(1, sizeof(speed_target_msg_t));
    }
    if (g_speed_state_queue == NULL) {
        g_speed_state_queue = xQueueCreate(1, sizeof(speed_service_state_t));
    }
    if (g_speed_target_queue == NULL || g_speed_state_queue == NULL) {
        return false;
    }

    g_low_speed_ff_enable = false;
    g_emergency_stop = false;
    speed_loop_core_init(&g_speed_core);
    encoder_reset();
    speed_service_publish_snapshot(0, 0);   /* 发布初始停稳快照 */
    return true;
}

bool speed_service_set_target_with_ff(int32_t left_rpm,
                                      int32_t right_rpm,
                                      bool low_speed_ff_enable)
{
    speed_target_msg_t target = {0};

    if (g_speed_target_queue == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    bool rejected = g_emergency_stop &&
                    (left_rpm != 0 || right_rpm != 0);
    taskEXIT_CRITICAL();
    if (rejected) {
        return false;
    }

    target.left_rpm = left_rpm;
    target.right_rpm = right_rpm;
    target.low_speed_ff_enable = low_speed_ff_enable;
    return (xQueueOverwrite(g_speed_target_queue, &target) == pdPASS);
}

/* 应用层公开接口：默认不启用 yaw 专用低速前馈。 */
bool speed_service_set_target(int32_t left_rpm, int32_t right_rpm)
{
    return speed_service_set_target_with_ff(left_rpm, right_rpm, false);
}

void speed_service_step_10ms(void)
{
    taskENTER_CRITICAL();
    bool emergency_stop = g_emergency_stop;
    taskEXIT_CRITICAL();

    if (emergency_stop) {
        speed_loop_core_set_target(&g_speed_core, 0, 0);
        tb6612_brake();

        encoder_data_t stopped_encoder;
        encoder_get_data(&stopped_encoder);
#if TB6612_REVERSE_DIRECTION
        stopped_encoder.left_count = -stopped_encoder.left_count;
        stopped_encoder.right_count = -stopped_encoder.right_count;
#endif
        speed_service_publish_snapshot(stopped_encoder.left_count,
                                       stopped_encoder.right_count);
        return;
    }

    /* CPR为0时无法形成有效负反馈，立即锁存停车而不是让PID饱和。 */
    uint32_t counts_per_rev = g_speed_core.params != NULL ?
        g_speed_core.params->counts_per_rev : 0U;
    if (counts_per_rev == 0U) {
        speed_service_emergency_stop();
        return;
    }

    /* 取最新速度目标；收到 0/0 时核心会立即停止并要求制动。 */
    speed_target_msg_t new_target;
    if (g_speed_target_queue != NULL &&
        xQueueReceive(g_speed_target_queue, &new_target, 0) == pdPASS) {
        g_low_speed_ff_enable = new_target.low_speed_ff_enable;
        if (speed_loop_core_set_target(&g_speed_core,
                                       new_target.left_rpm,
                                       new_target.right_rpm)) {
            tb6612_brake();
            /* 制动路径下尚未读编码器，沿用上次的累计计数。 */
            speed_service_state_t prev = {0};
            (void)speed_service_get_state(&prev);
            speed_service_publish_snapshot(prev.left_count, prev.right_count);
        }
    }

    /* 10ms 编码器采样，核心内部累计到 50ms 才做 PID。 */
    encoder_data_t encoder;
    encoder_get_data(&encoder);

    /* 电机方向整体反转时，反馈速度与累计里程坐标也必须反转。
     * 否则 PID 会成为正反馈，且固定距离控制无法到达目标。 */
#if TB6612_REVERSE_DIRECTION
    encoder.left_delta  = -encoder.left_delta;
    encoder.right_delta = -encoder.right_delta;
    encoder.left_count  = -encoder.left_count;
    encoder.right_count = -encoder.right_count;
#endif

    speed_loop_core_output_t output = {0};
    if (speed_loop_core_update(&g_speed_core,
                               encoder.left_delta,
                               encoder.right_delta,
                               SPEED_LOOP_CORE_SAMPLE_PERIOD_MS,
                               counts_per_rev,
                               g_low_speed_ff_enable,
                               &output)) {
        if (output.brake) {
            tb6612_brake();
        } else {
            tb6612_set_speed((int16_t)output.right_pwm,
                             (int16_t)output.left_pwm);
        }
    }

    /* 发布快照，携带编码器累计计数供距离控制使用。 */
    speed_service_publish_snapshot(encoder.left_count, encoder.right_count);
}

bool speed_service_get_state(speed_service_state_t *out)
{
    if (g_speed_state_queue == NULL || out == NULL ||
        xQueuePeek(g_speed_state_queue, out, 0) != pdPASS) {
        return false;
    }
    return true;
}

bool speed_service_wheels_stopped_snapshot(void)
{
    speed_service_state_t state = {0};
    if (!speed_service_get_state(&state)) {
        return false;
    }
    return state.stopped;
}

void speed_service_emergency_stop(void)
{
    taskENTER_CRITICAL();
    g_emergency_stop = true;
    g_low_speed_ff_enable = false;
    speed_loop_core_set_target(&g_speed_core, 0, 0);
    taskEXIT_CRITICAL();

    if (g_speed_target_queue != NULL) {
        (void)xQueueReset(g_speed_target_queue);
    }
    tb6612_brake();

    speed_service_state_t previous = {0};
    if (speed_service_get_state(&previous)) {
        speed_service_publish_snapshot(previous.left_count,
                                       previous.right_count);
    }
}

void speed_service_clear_emergency_stop(void)
{
    if (g_speed_target_queue != NULL) {
        (void)xQueueReset(g_speed_target_queue);
    }
    taskENTER_CRITICAL();
    speed_loop_core_set_target(&g_speed_core, 0, 0);
    g_low_speed_ff_enable = false;
    g_emergency_stop = false;
    taskEXIT_CRITICAL();
}

bool speed_service_is_emergency_stopped(void)
{
    bool stopped;
    taskENTER_CRITICAL();
    stopped = g_emergency_stop;
    taskEXIT_CRITICAL();
    return stopped;
}
