/**
 * @file line_tracking_service.c
 * @brief 灰度循迹的FreeRTOS与硬件适配层。
 *
 * @details
 * 本层只负责命令/状态队列、灰度采样、编码器定距、故障监督和速度目标
 * 下发。判线、PD、丢线重捕、弯道调速、左右轮合成与整形全部由
 * line_tracking_control纯算法完成。任务参数集中在 config/task_params.c，
 * 共享的传感器几何常量保留在 line_tracking_config.h。
 *
 * 每个10ms周期的数据流：
 * @code
 * command queue + gray_sensor + speed snapshot
 *       -> distance/safety checks
 *       -> line_tracking_control_update
 *       -> speed_service target queue
 *       -> state snapshot queue
 * @endcode
 * service不复制PID或弯道参数；profile只切换到当前菜单任务的 .line
 * 参数指针，Task2/4/5/6之间互不改写。
 */
#include "service/line_tracking_service.h"
#include "control/distance_control.h"
#include "control/line_tracking_config.h"
#include "control/line_tracking_control.h"
#include "service/speed_service.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

/** service内部一次性命令类型；长度1队列只保留调用者最新意图。 */
typedef enum {
    LINE_TRACK_CMD_START = 0,        /**< 开始一段新的定距循迹 */
    LINE_TRACK_CMD_START_CHECKPOINT, /**< 开始不定距的检查点循迹 */
    LINE_TRACK_CMD_ABORT             /**< 用户主动中止，不作为故障 */
} line_tracking_cmd_type_t;

/** 长度1命令队列中的消息；START一次性携带全部循线运行参数。 */
typedef struct {
    line_tracking_cmd_type_t type;
    int32_t target_counts; /* 编码器目标距离 */
    int32_t base_rpm;      /* 两轮公共基础速度，不包含循线差速 */
} line_tracking_cmd_t;

/* ========================================================================== */
/* service私有对象：只有本模块的10ms入口会修改，其他任务通过队列访问         */
/* ========================================================================== */

static QueueHandle_t g_command_queue = NULL; /**< START/ABORT长度1覆盖队列 */
static QueueHandle_t g_state_queue = NULL;   /**< 最新状态长度1覆盖队列 */
static line_tracking_control_t g_line_control; /**< 纯循迹控制器实例 */
static distance_control_t g_distance_control; /**< 编码器距离监督实例 */
static line_tracking_service_state_t g_state; /**< 下次发布的状态工作副本 */
static int32_t g_last_average_count = 0;      /**< 上周期左右累计计数平均值 */
static int32_t g_checkpoint_start_avg = 0;    /**< 检查点模式编码起点 */
static int32_t g_checkpoint_travelled = 0;    /**< 检查点模式累计行程 */
static uint16_t g_no_progress_ticks = 0U;     /**< 连续无编码器进度周期 */
static uint32_t g_run_ticks = 0U;             /**< 本次运行总10ms周期 */
static bool g_checkpoint_mode = false;         /**< true=不以编码距离判停 */
static volatile int32_t g_speed_cap_rpm = 0;   /**< 0=不限速 */
static line_tracking_profile_t g_profile =
    LINE_TRACKING_PROFILE_TASK4_COMBINED;
/** service层当前任务参数；仅在停车并切换profile时改指针。 */
static volatile line_control_params_t *g_line_params = &g_task4_params.line;

/** 左右编码器累计计数取平均，使用int64_t避免求和溢出。 */
static int32_t average_count(const speed_service_state_t *speed)
{
    return (int32_t)(((int64_t)speed->left_count + speed->right_count) / 2);
}

static int32_t integer_sqrt_i32(int32_t value)
{
    if (value <= 0) {
        return 0;
    }
    int32_t result = 0;
    int32_t bit = 1 << 30;
    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static void apply_speed_cap(line_tracking_output_t *line)
{
    int32_t cap = g_speed_cap_rpm;
    if (line == NULL || line->base_rpm <= 0 || cap <= 0 ||
        cap >= line->base_rpm) {
        return;
    }
    line->left_target_rpm = (int32_t)(
        (int64_t)line->left_target_rpm * cap / line->base_rpm);
    line->right_target_rpm = (int32_t)(
        (int64_t)line->right_target_rpm * cap / line->base_rpm);
    line->turn_rpm = (int32_t)(
        (int64_t)line->turn_rpm * cap / line->base_rpm);
    line->requested_turn_rpm = (int32_t)(
        (int64_t)line->requested_turn_rpm * cap / line->base_rpm);
    line->base_rpm = cap;
}

static void apply_distance_deceleration(line_tracking_output_t *line)
{
    if (line == NULL || line->base_rpm <= 0) {
        return;
    }

    int32_t distance_to_brake =
        g_distance_control.remaining - g_line_params->brake_margin_counts;
    if (distance_to_brake >= g_line_params->distance_decel_counts) {
        return;
    }
    if (distance_to_brake < 0) {
        distance_to_brake = 0;
    }

    int32_t coefficient = g_line_params->distance_decel_coefficient;
    if (coefficient < 0) {
        coefficient = 0;
    }
    int64_t scaled_distance = (int64_t)distance_to_brake * coefficient;
    if (scaled_distance > INT32_MAX) {
        scaled_distance = INT32_MAX;
    }
    int32_t cap_rpm = integer_sqrt_i32((int32_t)scaled_distance);
    if (cap_rpm < g_line_params->distance_min_rpm) {
        cap_rpm = g_line_params->distance_min_rpm;
    }
    if (cap_rpm >= line->base_rpm) {
        return;
    }

    line->left_target_rpm = (int32_t)(
        (int64_t)line->left_target_rpm * cap_rpm / line->base_rpm);
    line->right_target_rpm = (int32_t)(
        (int64_t)line->right_target_rpm * cap_rpm / line->base_rpm);
    line->turn_rpm = (int32_t)(
        (int64_t)line->turn_rpm * cap_rpm / line->base_rpm);
    line->requested_turn_rpm = (int32_t)(
        (int64_t)line->requested_turn_rpm * cap_rpm / line->base_rpm);
    line->base_rpm = cap_rpm;
}

/**
 * @brief 汇总传感器、control和距离状态，并原子覆盖最新快照。
 * @details 灰度原始值/强度始终更新，便于停车时观察传感器；只有active且
 * output_valid时才覆盖base/turn等运动输出，防止stop_tracking刚清零后
 * 又被本函数收到的旧局部line结果恢复成非零。
 */
static void publish_state(const gray_sensor_data_t *sensor,
                          const line_tracking_output_t *line)
{
    if (sensor != NULL) {
        memcpy(g_state.raw, sensor->raw, sizeof(g_state.raw));
        memcpy(g_state.normalized, sensor->normalized,
               sizeof(g_state.normalized));
        g_state.digital_mask = sensor->digital_mask;
    }
    if (line != NULL) {
        g_state.line_position = line->position;
        /* stop_tracking已清零输出时，不允许局部line覆盖停车快照。 */
        if (g_state.active && g_state.output_valid) {
            g_state.steering_error = line->steering_error;
            g_state.base_rpm = line->base_rpm;
            g_state.turn_rpm = line->turn_rpm;
            g_state.requested_turn_rpm = line->requested_turn_rpm;
            g_state.reacquire_blending = line->reacquire_blending;
        }
        g_state.line_strength = line->line_strength;
        g_state.line_peak_strength = line->line_peak_strength;
        g_state.strong_channel_count = line->strong_channel_count;
        g_state.sensor_valid = line->sensor_valid;
        g_state.line_detected = line->line_detected;
        g_state.checkpoint_detected = line->checkpoint_detected;
        g_state.checkpoint_count = line->checkpoint_count;
    }

    if (g_checkpoint_mode) {
        g_state.target_counts = 0;
        g_state.travelled_counts = g_checkpoint_travelled;
        g_state.remaining_counts = 0;
    } else {
        g_state.target_counts = g_distance_control.target_counts;
        g_state.travelled_counts = g_distance_control.travelled;
        g_state.remaining_counts = g_distance_control.remaining;
    }
    (void)xQueueOverwrite(g_state_queue, &g_state);
}

/**
 * @brief 统一处理正常到达、用户中止和故障退出。
 * @param fault   NONE=正常退出；其他值先锁存speed_service ESTOP
 * @param reached true=达到定距目标，保留distance reached状态
 * @param aborted true=用户主动中止
 *
 * @note 只在原状态active时写一次(0,0)。停止态不能每10ms持续写零，
 * 否则会覆盖其他运动模式之后下发的手动目标。
 */
static void stop_tracking(line_tracking_fault_t fault,
                          bool reached,
                          bool aborted)
{
    bool was_active = g_state.active;
    if (fault != LINE_TRACK_FAULT_NONE) {
        speed_service_emergency_stop();
    }
    if (was_active) {
        (void)speed_service_set_target(0, 0);
    }
    if (!reached) {
        distance_control_abort(&g_distance_control);
    }
    line_tracking_control_reset(&g_line_control);
    g_state.active = false;
    g_state.reached = reached;
    g_state.aborted = aborted;
    g_state.fault = fault;
    g_state.steering_error = 0;
    g_state.base_rpm = 0;
    g_state.turn_rpm = 0;
    g_state.requested_turn_rpm = 0;
    g_state.reacquire_blending = false;
    g_state.checkpoint_detected = false;
    g_state.checkpoint_count = 0;
    g_state.output_valid = false;
}

/* ========================================================================== */
/* 对外生命周期与命令API                                                    */
/* ========================================================================== */

bool line_tracking_service_init(void)
{
    if (g_command_queue == NULL) {
        g_command_queue = xQueueCreate(1, sizeof(line_tracking_cmd_t));
    }
    if (g_state_queue == NULL) {
        g_state_queue = xQueueCreate(1, sizeof(line_tracking_service_state_t));
    }
    if (g_command_queue == NULL || g_state_queue == NULL) {
        return false;
    }

    line_tracking_control_init(&g_line_control);
    distance_control_init(&g_distance_control);
    memset(&g_state, 0, sizeof(g_state));
    g_last_average_count = 0;
    g_checkpoint_start_avg = 0;
    g_checkpoint_travelled = 0;
    g_no_progress_ticks = 0U;
    g_run_ticks = 0U;
    g_checkpoint_mode = false;
    g_speed_cap_rpm = 0;
    publish_state(NULL, NULL);
    return true;
}

bool line_tracking_service_select_profile(line_tracking_profile_t profile)
{
    if (line_tracking_service_is_active_snapshot()) {
        return false;
    }
    if (profile == LINE_TRACKING_PROFILE_TASK2_LAP) {
        g_line_params = &g_task2_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK4_COMBINED) {
        g_line_params = &g_task4_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK5_LAP_BALL) {
        g_line_params = &g_task5_params.line;
    } else if (profile == LINE_TRACKING_PROFILE_TASK6_SPECIFIED_BALL) {
        g_line_params = &g_task6_params.lap.line;
    } else {
        return false;
    }
    line_tracking_control_select_profile(&g_line_control, profile);
    g_profile = profile;
    return true;
}

bool line_tracking_service_start(int32_t target_counts,
                                 int32_t base_rpm)
{
    line_tracking_cmd_t command = {
        .type = LINE_TRACK_CMD_START,
        .target_counts = target_counts,
        .base_rpm = base_rpm,
    };

    if (g_command_queue == NULL || target_counts <= 0 || base_rpm <= 0 ||
        base_rpm > LINE_TRACKING_MAX_BASE_RPM ||
        line_tracking_service_is_active_snapshot()) {
        return false;
    }
    return (xQueueOverwrite(g_command_queue, &command) == pdPASS);
}

bool line_tracking_service_start_checkpoint(int32_t base_rpm)
{
    line_tracking_cmd_t command = {
        .type = LINE_TRACK_CMD_START_CHECKPOINT,
        .target_counts = 0,
        .base_rpm = base_rpm,
    };

    if (g_command_queue == NULL || base_rpm <= 0 ||
        base_rpm > LINE_TRACKING_MAX_BASE_RPM ||
        line_tracking_service_is_active_snapshot()) {
        return false;
    }
    return (xQueueOverwrite(g_command_queue, &command) == pdPASS);
}

bool line_tracking_service_set_speed_cap(int32_t cap_rpm)
{
    if (cap_rpm < 0 || cap_rpm > LINE_TRACKING_MAX_BASE_RPM) {
        return false;
    }
    g_speed_cap_rpm = cap_rpm;
    return true;
}

bool line_tracking_service_abort(void)
{
    line_tracking_cmd_t command = {.type = LINE_TRACK_CMD_ABORT};
    if (g_command_queue == NULL) {
        return false;
    }
    return (xQueueOverwrite(g_command_queue, &command) == pdPASS);
}

/* ========================================================================== */
/* 10ms主状态机：命令 -> 安全/距离 -> control -> 速度下发 -> 快照发布        */
/* ========================================================================== */

void line_tracking_service_step_10ms(void)
{
    gray_sensor_data_t sensor = {0};
    line_tracking_output_t line = {0};
    speed_service_state_t speed = {0};
    /* 每周期只采样一次；后续所有判断和control使用同一份时间一致快照。 */
    bool sensor_ok = gray_sensor_read(&sensor);
    bool speed_ok = speed_service_get_state(&speed);
    line_tracking_cmd_t command;

    /* 每周期默认无有效输出，禁止安全仲裁误认上一周期目标仍有效。 */
    g_state.turn_rpm = 0;
    g_state.output_valid = false;

    /* 阶段1：先消费一次性命令。START只有通过速度、ADC和标定检查后
     * 才启动distance/control，确保第一帧不会在无有效输入时驱动车轮。 */
    if (g_command_queue != NULL &&
        xQueueReceive(g_command_queue, &command, 0) == pdPASS) {
        if (command.type == LINE_TRACK_CMD_ABORT) {
            stop_tracking(LINE_TRACK_FAULT_NONE, false, true);
        } else if (!speed_ok) {
            stop_tracking(LINE_TRACK_FAULT_SPEED_STATE, false, false);
        } else if (!sensor.adc_ok) {
            stop_tracking(LINE_TRACK_FAULT_ADC, false, false);
        } else if (!sensor_ok || sensor.valid_mask != 0xFFU ||
                   !GRAY_SENSOR_CALIBRATION_READY) {
            stop_tracking(LINE_TRACK_FAULT_CALIBRATION, false, false);
        } else {
            g_checkpoint_mode =
                command.type == LINE_TRACK_CMD_START_CHECKPOINT;
            if (g_checkpoint_mode) {
                distance_control_init(&g_distance_control);
                line_tracking_control_start_checkpoint(
                    &g_line_control, command.base_rpm, 0U);
            } else {
                /* 距离状态机只监督进度，不生成直行速度。 */
                distance_control_start_progress(
                    &g_distance_control,
                    command.target_counts,
                    g_line_params->brake_margin_counts,
                    average_count(&speed));
                if (g_profile == LINE_TRACKING_PROFILE_TASK2_LAP) {
                    line_tracking_control_start_checkpoint(
                        &g_line_control, command.base_rpm, 0U);
                } else {
                    line_tracking_control_start(&g_line_control,
                                                command.base_rpm);
                }
            }
            g_speed_cap_rpm = 0;
            g_last_average_count = average_count(&speed);
            g_checkpoint_start_avg = g_last_average_count;
            g_checkpoint_travelled = 0;
            g_no_progress_ticks = 0U;
            g_run_ticks = 0U;
            g_state.active = g_checkpoint_mode ||
                distance_control_is_active(&g_distance_control);
            g_state.reached = false;
            g_state.aborted = false;
            g_state.fault = LINE_TRACK_FAULT_NONE;
            g_state.base_rpm = command.base_rpm;
        }
    }

    /* 阶段2：active时按fail-fast顺序检查安全条件。任一分支停车后，
     * 本周期不会继续调用control或下发非零目标。 */
    if (g_state.active) {
        int32_t current_average = speed_ok ? average_count(&speed) : 0;
        if (g_checkpoint_mode && speed_ok) {
            int64_t travelled = (int64_t)current_average -
                                g_checkpoint_start_avg;
            if (travelled < 0) {
                travelled = -travelled;
            }
            g_checkpoint_travelled =
                travelled > INT32_MAX ? INT32_MAX : (int32_t)travelled;
        }
        if (!speed_ok) {
            stop_tracking(LINE_TRACK_FAULT_SPEED_STATE, false, false);
        } else if (++g_run_ticks > g_line_params->max_run_ticks) {
            stop_tracking(LINE_TRACK_FAULT_PROGRESS, false, false);
        } else if (current_average == g_last_average_count &&
                   g_state.base_rpm != 0 &&
                   ++g_no_progress_ticks >
                       g_line_params->no_progress_limit_ticks) {
            stop_tracking(LINE_TRACK_FAULT_PROGRESS, false, false);
        } else if (!g_checkpoint_mode &&
                   distance_control_update_progress(
                       &g_distance_control, current_average)) {
            stop_tracking(LINE_TRACK_FAULT_NONE, true, false);
        } else if (!sensor.adc_ok) {
            stop_tracking(LINE_TRACK_FAULT_ADC, false, false);
        } else if (!sensor_ok || sensor.valid_mask != 0xFFU ||
                   !GRAY_SENSOR_CALIBRATION_READY) {
            stop_tracking(LINE_TRACK_FAULT_CALIBRATION, false, false);
        } else {
            /* 阶段3：输入与距离均安全，才让纯control推进一个10ms周期。 */
            line_tracking_control_update(&g_line_control, &sensor, &line);
            if (line.stop_required) {
                if (!g_checkpoint_mode &&
                    g_profile == LINE_TRACKING_PROFILE_TASK4_COMBINED &&
                    line.checkpoint_count >=
                        g_line_params->checkpoint_stop_count) {
                    stop_tracking(LINE_TRACK_FAULT_NONE, true, false);
                } else {
                    stop_tracking(line.sensor_valid ? LINE_TRACK_FAULT_LOST :
                                  LINE_TRACK_FAULT_CALIBRATION,
                                  false, false);
                }
            } else {
                /* 检查点模式不应用编码距离减速。 */
                if (!g_checkpoint_mode) {
                    apply_distance_deceleration(&line);
                }
                apply_speed_cap(&line);
                g_state.output_valid = true;
                if (!speed_service_set_target(line.left_target_rpm,
                                              line.right_target_rpm)) {
                    stop_tracking(LINE_TRACK_FAULT_SPEED_STATE,
                                  false, false);
                }
            }
        }

        /* 只有本周期仍active才更新进度基准；停车路径保留最终计数快照。 */
        if (g_state.active && speed_ok) {
            if (current_average != g_last_average_count) {
                g_no_progress_ticks = 0U;
            }
            g_last_average_count = current_average;
        }
    } else if (GRAY_SENSOR_CALIBRATION_READY) {
        /* 停止态只做无状态预览，不推进PD、丢线或轮速整形状态。 */
        line_tracking_control_preview(&g_line_control, &sensor, &line);
    }

    /* 阶段4：无论运行、停车还是故障，本周期最后都发布一次一致快照。 */
    publish_state(&sensor, &line);
}

bool line_tracking_service_get_state(line_tracking_service_state_t *out)
{
    if (g_state_queue == NULL || out == NULL ||
        xQueuePeek(g_state_queue, out, 0) != pdPASS) {
        return false;
    }
    return true;
}

bool line_tracking_service_is_active_snapshot(void)
{
    line_tracking_service_state_t state;
    return line_tracking_service_get_state(&state) && state.active;
}
