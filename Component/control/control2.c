/**
 * @file    control2.c
 * @brief   H题六任务中的循线、管球和组合运动状态机。
 *
 * @details task2_lap_* 是菜单Task2的一圈循线状态机；ball_control_*
 * 是菜单Task3~6共用的管球算法内核。所有可调默认值统一在
 * config/task_params.c，结构和单位说明在 config/task_params.h。
 */

#include "control/control2.h"
#include "service/line_tracking_service.h"
#include "service/speed_service.h"
#include "stepper/Emm_V5.h"
#include "maixcam/maixcam.h"
#include "common/util.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>


/* ═══════════════════════════════════════════════════════════════════════════
 *  官方任务2 — 单纯巡线一圈
 *
 *  赛道: AB/CD=1.5m直道, BC/DA=0.5m半径半圆弧, 周长≈6.14m
 *  要求: ≤20s完成, 停车偏差≤2cm, A点有5cm垂直启停线
 *
 *  控制调用链：
 *    task2_lap_step()                            ← 10ms控制任务
 *      ├─ line_tracking_service_step_10ms()      ← 循线服务
 *      ├─ speed_service_step_10ms()              ← 速度内环
 *      └─ 灰度启停线状态机（见下）
 *
 *  状态流: IDLE → TRACKING → BRAKING → COMPLETE/FAULT
 *
 *  ── 参数调节 ──
 *  菜单Task2全部参数位于 g_task2_params：顶层是速度和终点检测，
 *  .line 是独立循线参数，.speed 是独立轮速PID/编码器参数。
 *
 *  终点检测流程：
 *    1. 启动后500ms内跳过起点横线（grace period）
 *    2. 连续50ms既不在横线又正常循线 → 解锁终点（checkpoint_armed）
 *    3. 解锁后首次≥4路强黑 → 同周期锁存TB6612硬件制动
 *    4. 两轮停稳 → COMPLETE
 * ═══════════════════════════════════════════════════════════════════════════ */

/* 菜单Task2的参数统一位于 g_task2_params。这里保留语义化别名，
 * 使状态机代码可读，同时保证所有默认值只在 task_params.c 出现一次。 */
#define TASK2_LAP_BASE_RPM           (g_task2_params.base_rpm)
#define TASK2_LAP_TARGET_COUNTS      (g_task2_params.virtual_target_counts)
#define TASK2_LAP_LINE_DETECT_FRAMES (g_task2_params.line_detect_frames)
#define TASK2_LAP_LINE_DETECT_STRONG (g_task2_params.line_detect_strong)
#define TASK2_LAP_LINE_CLEAR_FRAMES  (g_task2_params.line_clear_frames)
#define TASK2_LAP_START_GRACE_TICKS  (g_task2_params.start_grace_ticks)

/* ── 模块状态 ── */
static task2_lap_state_t g_task2_lap_state = TASK2_LAP_IDLE;
static uint32_t      g_task2_lap_start_tick = 0;
static uint32_t      g_task2_lap_total_ms = 0;
static uint8_t       g_task2_lap_detect_cnt = 0;
static uint8_t       g_task2_lap_clear_cnt = 0;
static uint16_t      g_task2_lap_grace_cnt = 0;
static uint8_t       g_task2_lap_mask = 0xFF;
static uint8_t       g_task2_lap_strong = 0;
static uint8_t       g_task2_lap_fault = 0;
static bool          g_task2_lap_checkpoint_armed = false;

bool task2_lap_start(void)
{
    if (task2_lap_is_active() || line_tracking_service_is_active_snapshot()) {
        return false;
    }

    speed_service_emergency_stop();
    if (!speed_service_select_profile(SPEED_PROFILE_TASK2_LAP) ||
        !line_tracking_service_select_profile(
            LINE_TRACKING_PROFILE_TASK2_LAP)) {
        return false;
    }
    speed_service_clear_emergency_stop();
    if (!line_tracking_service_start(TASK2_LAP_TARGET_COUNTS, TASK2_LAP_BASE_RPM)) {
        speed_service_emergency_stop();
        return false;
    }

    g_task2_lap_state      = TASK2_LAP_TRACKING;
    g_task2_lap_start_tick = xTaskGetTickCount();
    g_task2_lap_total_ms   = 0;
    g_task2_lap_detect_cnt = 0;
    g_task2_lap_clear_cnt  = 0;
    g_task2_lap_grace_cnt  = 0;
    g_task2_lap_mask       = 0xFF;
    g_task2_lap_strong     = 0;
    g_task2_lap_fault      = 0;
    g_task2_lap_checkpoint_armed = false;
    return true;
}

void task2_lap_stop(void)
{
    (void)line_tracking_service_abort();
    speed_service_emergency_stop();
    g_task2_lap_state = TASK2_LAP_IDLE;
}

bool task2_lap_step(uint32_t dt_ms)
{
    (void)dt_ms;
    if (g_task2_lap_state == TASK2_LAP_IDLE || g_task2_lap_state == TASK2_LAP_COMPLETE ||
        g_task2_lap_state == TASK2_LAP_FAULT)
        return false;

    line_tracking_service_step_10ms();
    speed_service_step_10ms();

    line_tracking_service_state_t lt = {0};
    bool ok = line_tracking_service_get_state(&lt);
    bool checkpoint = ok &&
                      lt.strong_channel_count >= TASK2_LAP_LINE_DETECT_STRONG;
    g_task2_lap_mask   = ok ? lt.digital_mask : 0xFF;
    g_task2_lap_strong = ok ? lt.strong_channel_count : 0;

    switch (g_task2_lap_state) {
    case TASK2_LAP_TRACKING:
        g_task2_lap_total_ms = (uint32_t)(xTaskGetTickCount() - g_task2_lap_start_tick) *
                        portTICK_PERIOD_MS;
        if (!ok || lt.fault != LINE_TRACK_FAULT_NONE ||
            (!lt.active && !lt.aborted)) {
            g_task2_lap_fault = ok ? (uint8_t)lt.fault :
                              (uint8_t)LINE_TRACK_FAULT_SPEED_STATE;
            speed_service_emergency_stop();
            g_task2_lap_state = TASK2_LAP_FAULT;
            break;
        }

        if (g_task2_lap_grace_cnt < TASK2_LAP_START_GRACE_TICKS) {
            g_task2_lap_grace_cnt++;
            g_task2_lap_clear_cnt = 0;
            g_task2_lap_detect_cnt = 0;
        } else if (!g_task2_lap_checkpoint_armed) {
            if (!checkpoint && lt.line_detected) {
                if (++g_task2_lap_clear_cnt >= TASK2_LAP_LINE_CLEAR_FRAMES) {
                    g_task2_lap_checkpoint_armed = true;
                }
            } else {
                g_task2_lap_clear_cnt = 0;
            }
        } else if (checkpoint) {
            if (++g_task2_lap_detect_cnt >= TASK2_LAP_LINE_DETECT_FRAMES) {
                (void)line_tracking_service_abort();
                /* 当周期直接锁存硬件制动，不等待下一周期消费零速队列。 */
                speed_service_emergency_stop();
                g_task2_lap_state = TASK2_LAP_BRAKING;
            }
        } else {
            g_task2_lap_detect_cnt = 0;
        }
        break;
    case TASK2_LAP_BRAKING:
        if (speed_service_wheels_stopped_snapshot() &&
            !line_tracking_service_is_active_snapshot()) {
            g_task2_lap_state = TASK2_LAP_COMPLETE;
        }
        break;
    case TASK2_LAP_COMPLETE:
    case TASK2_LAP_FAULT:
        break;
    default:
        g_task2_lap_state = TASK2_LAP_IDLE;
        break;
    }
    return true;
}

bool task2_lap_is_active(void)
{
    return g_task2_lap_state == TASK2_LAP_TRACKING ||
           g_task2_lap_state == TASK2_LAP_BRAKING;
}

bool task2_lap_get_status(task2_lap_status_t *out)
{
    if (!out) return false;
    out->state        = g_task2_lap_state;
    out->total_time_ms = g_task2_lap_total_ms;
    out->digital_mask = g_task2_lap_mask;
    out->strong_channel = g_task2_lap_strong;
    out->lap_detected = (g_task2_lap_state == TASK2_LAP_BRAKING ||
                         g_task2_lap_state == TASK2_LAP_COMPLETE);
    out->checkpoint_armed = g_task2_lap_checkpoint_armed;
    out->fault = g_task2_lap_fault;
    return true;
}

const char *task2_lap_state_name(task2_lap_state_t s)
{
    switch (s) {
    case TASK2_LAP_IDLE:        return "IDLE";
    case TASK2_LAP_TRACKING:   return "TRACK";
    case TASK2_LAP_BRAKING:    return "BRAKE";
    case TASK2_LAP_COMPLETE:   return "DONE";
    case TASK2_LAP_FAULT:      return "FAULT";
    default:               return "????";
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  管球控制内核 — 被菜单Task3、Task4、Task5、Task6共用
 *
 *  控制策略: 串级控制 (外环位置PID + 内环电机角度伺服)
 *    摄像头 → 钢珠X坐标 → 误差 → 期望速度 → 误差 → 倾角(脉冲)
 *      → Emm_V5 位置控制 → 步进电机 → 管道倾斜 → 钢珠滚动
 *
 *  状态流: IDLE → HOMING → TRACKING ⇄ LOST
 *
 *  ── 参数调节 ──
 *  菜单Task3 → g_task3_params.ball
 *  菜单Task4 → g_task4_params.ball
 *  菜单Task5 → g_task5_params.ball
 *  菜单Task6 → g_task6_params.lap.ball
 *  共享机械标定 → g_hardware_calibration
 *  每个字段的单位、增减效果和安全约束见 config/task_params.h。
 * ═══════════════════════════════════════════════════════════════════════════ */

/* 当前运行的菜单任务参数。切换只允许发生在任务启动/调平阶段；控制周期
 * 仅读取该任务独立参数块。共享机械零点、方向和硬限位则读取
 * g_hardware_calibration，并在每次电机命令前作为最后一道安全限制。 */
static volatile ball_control_params_t *g_ball_params = &g_task3_params.ball;

/* 以下语义化别名均直接指向当前菜单任务的参数块，不产生参数副本。 */
#define g_ball_target_x                 (g_ball_params->default_target_x)
#define g_ball_max_tilt                 (g_ball_params->max_tilt_pos)
#define g_ball_max_tilt_neg             (g_ball_params->max_tilt_neg)
#define g_ball_tilt_sign                (g_hardware_calibration.tilt_sign)
#define g_ball_kp_pos                   (g_ball_params->kp_pos_milli)
#define g_ball_kp_vel                   (g_ball_params->kp_vel_milli)
#define g_ball_max_desired_vel          (g_ball_params->max_desired_vel_px_s)
#define g_ball_dead_zone                (g_ball_params->dead_zone_pulses)
#define g_ball_dead_boost               (g_ball_params->dead_boost_pulses)
#define g_ball_pos_static_tilt          (g_ball_params->pos_static_tilt)
#define g_ball_dead_vel_th              (g_ball_params->dead_vel_px_s)
#define g_ball_tolerance_px             (g_ball_params->tolerance_px)
#define g_ball_reached_vel_px_s         (g_ball_params->reached_vel_px_s)
#define g_ball_stable_time_ms           (g_ball_params->stable_time_ms)
#define g_ball_vel_filter_alpha_milli   (g_ball_params->vel_filter_alpha_milli)
#define g_ball_output_slew_pulses       (g_ball_params->output_slew_pulses)
#define g_ball_motor_offset             (g_hardware_calibration.motor_zero_offset_pulses)
#define g_ball_hold_center_x            (g_ball_params->hold_center_x)
#define g_ball_hold_gain_milli          (g_ball_params->hold_gain_milli)
#define BALL_CONTROL_LOST_TIMEOUT_MS         (g_ball_params->lost_timeout_ms)
#define BALL_CONTROL_EDGE_LOW_X              (g_ball_params->edge_low_x)
#define BALL_CONTROL_EDGE_HIGH_X             (g_ball_params->edge_high_x)
#define BALL_CONTROL_EDGE_RECOVER_TILT       (g_ball_params->edge_recover_tilt)
#define BALL_CONTROL_EDGE_RECOVER_MS         (g_ball_params->edge_recover_ms)
#define BALL_CONTROL_POS_KICK_FAR_MS         (g_ball_params->pos_kick_far_ms)
#define BALL_CONTROL_POS_KICK_NEAR_MS        (g_ball_params->pos_kick_near_ms)
#define BALL_CONTROL_POS_KICK_NEAR_TILT      (g_ball_params->pos_kick_near_tilt)
#define BALL_CONTROL_POS_KICK_FAR_ERROR      (g_ball_params->pos_kick_far_error_px)
#define BALL_CONTROL_POS_KICK_COOLDOWN_MS    (g_ball_params->pos_kick_cooldown_ms)
#define BALL_CONTROL_HOMING_TIME_MS          (g_ball_params->homing_time_ms)

/* ── 内部状态 ── */
struct {
    ball_control_state_t state;
    uint32_t      timer_ms;
    uint32_t      stable_ms;
    uint32_t      no_frame_ms;
    uint32_t      last_frame_id;
    int32_t       target_x;
    int32_t       motor_target;
    int32_t       last_error;
    int32_t       last_ball_x;       /**< 最后一次有效钢珠位置 */
    int32_t       last_ball_vel;     /**< 滤波后球速 (px/s) */
    int32_t       lost_recover_tilt; /**< 丢球后的有界回收倾角 */
    uint32_t      pos_kick_ms;       /**< 正方向启动脉冲剩余时间 */
    uint32_t      pos_kick_cooldown_ms;
    int32_t       pos_kick_tilt;
    int32_t       desired_vel;       /**< 外环期望球速 (px/s) */
    int32_t       velocity_error;    /**< 内环速度误差 (px/s) */
    bool          first_frame;
    bool          reached;
} g_ball_control;

static int32_t g_ball_control_external_tilt_ff = 0;
static task5_lap_mode_t g_ball_control_lap_mode = 0;

/* ── 内部辅助 ── */

static int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ── 位置保持倾角查表插值 ──
 * 标定表使能且有≥2个点时，根据 target_x 线性插值得到保持倾角；
 * 否则回退到线性公式 hold_gain × (target_x - center_x) / 1000。 */
static int32_t ball_control_lookup_hold_tilt(int32_t target_x)
{
    int32_t n = g_ball_params->hold_cal_count;
    if (n > (int32_t)TASK_HOLD_CAL_POINT_COUNT) {
        n = TASK_HOLD_CAL_POINT_COUNT;
    }
    if (g_ball_params->hold_cal_enabled && n >= 2) {
        if (target_x <= g_ball_params->hold_cal_x[0]) {
            return g_ball_params->hold_cal_tilt[0];
        }
        if (target_x >= g_ball_params->hold_cal_x[n - 1]) {
            return g_ball_params->hold_cal_tilt[n - 1];
        }
        for (int32_t i = 0; i < n - 1; i++) {
            int32_t x0 = g_ball_params->hold_cal_x[i];
            int32_t x1 = g_ball_params->hold_cal_x[i + 1];
            if (target_x >= x0 && target_x <= x1) {
                int32_t t0 = g_ball_params->hold_cal_tilt[i];
                int32_t t1 = g_ball_params->hold_cal_tilt[i + 1];
                if (x1 <= x0) {
                    continue;
                }
                return t0 + (t1 - t0) * (target_x - x0) / (x1 - x0);
            }
        }
    }
    return (int32_t)((int64_t)g_ball_hold_gain_milli *
                     (target_x - g_ball_hold_center_x) / 1000);
}

static void ball_control_motor_goto(int32_t pulses)
{
    /* 第1层：软限位约束“相对调平位置的任务倾角”。零点微调不能改变
     * 软限位语义，否则非零offset会无意压缩一侧任务控制范围。 */
    int32_t soft_pos = g_ball_max_tilt;
    int32_t soft_neg = g_ball_max_tilt_neg;
    if (soft_pos < 0) soft_pos = 0;
    if (soft_neg < 0) soft_neg = 0;
    pulses = clamp_i32(pulses, -soft_neg, soft_pos);

    /* 第2层：叠加小量水平零点微调；用64位中间量防Watch误值溢出。 */
    pulses = util_clamp_i64_to_i32(
        (int64_t)pulses + g_ball_motor_offset);

    /* 第3层：按电机绝对零点坐标应用机械硬限位。即使任务软限位或
     * offset误填，最终位置也不能越过已标定的机构安全行程。 */
    int32_t hard_pos = g_hardware_calibration.hard_max_tilt_pos;
    int32_t hard_neg = g_hardware_calibration.hard_max_tilt_neg;
    if (hard_pos < 0) hard_pos = 0;
    if (hard_neg < 0) hard_neg = 0;
    pulses = clamp_i32(pulses, -hard_neg, hard_pos);
    g_ball_control.motor_target = pulses;

    /* 用完整版位置控制(0xFD)替代快捷版(0xFC):
     * 0xFC 把 int32_t 负值直接发 4 字节, 电机端可能按无符号解读导致超限;
     * 0xFD 用 dir 方向 + uint32_t 脉冲数, 不会出现负值解析问题。 */
    uint8_t  dir  = (pulses < 0) ? 1 : 0;
    uint32_t abs_clk = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;
    Emm_V5_Pos_Control(g_hardware_calibration.stepper_addr, dir,
                       g_ball_params->motor_vel_rpm,
                       g_ball_params->motor_acc,
                       abs_clk, g_ball_params->motor_raf, false);
}

static void ball_control_motor_level(void)
{
    ball_control_motor_goto(0);
}

static void ball_control_reset_control(void)
{
    g_ball_control.stable_ms      = 0;
    g_ball_control.reached        = false;
    g_ball_control.first_frame    = true;
    g_ball_control.no_frame_ms    = 0;
    g_ball_control.last_error     = 0;
    g_ball_control.last_ball_x    = -1;
    g_ball_control.last_ball_vel  = 0;
    g_ball_control.lost_recover_tilt = 0;
    g_ball_control.pos_kick_ms = 0;
    g_ball_control.pos_kick_cooldown_ms = 0;
    g_ball_control.pos_kick_tilt = 0;
    g_ball_control.desired_vel    = 0;
    g_ball_control.velocity_error = 0;
}

static void ball_control_enter_lost(void)
{
    if (g_ball_control.last_ball_x >= BALL_CONTROL_EDGE_HIGH_X ||
        (g_ball_control.target_x > g_ball_control.last_ball_x &&
         g_ball_control.motor_target > 0)) {
        g_ball_control.lost_recover_tilt = -BALL_CONTROL_EDGE_RECOVER_TILT;
    } else if ((g_ball_control.last_ball_x >= 0 &&
                g_ball_control.last_ball_x <= BALL_CONTROL_EDGE_LOW_X) ||
               (g_ball_control.target_x < g_ball_control.last_ball_x &&
                g_ball_control.motor_target < 0)) {
        g_ball_control.lost_recover_tilt = BALL_CONTROL_EDGE_RECOVER_TILT;
    } else {
        g_ball_control.lost_recover_tilt = 0;
    }

    ball_control_motor_goto(g_ball_control.lost_recover_tilt);
    g_ball_control.state = BALL_CONTROL_LOST;
    g_ball_control.timer_ms = 0;
}

/* ── 公开接口 ── */

void ball_control_prepare_level_for_menu(uint8_t menu_task)
{
    /* 仅在任务未运行时切换整块参数。数字相同的参数也保持独立存储。 */
    if (menu_task == 4U) {
        g_ball_params = &g_task4_params.ball;
    } else if (menu_task == 5U) {
        g_ball_params = &g_task5_params.ball;
    } else if (menu_task == 6U) {
        g_ball_params = &g_task6_params.lap.ball;
    } else {
        g_ball_params = &g_task3_params.ball;
    }

    g_ball_control_lap_mode = 0;
    /* 初始化期间保持IDLE，防止控制任务与阻塞式配置命令并发。 */
    g_ball_control.state          = BALL_CONTROL_IDLE;
    g_ball_control.timer_ms       = 0;
    g_ball_control.target_x       = g_ball_target_x;
    g_ball_control.motor_target   = 0;
    g_ball_control_external_tilt_ff = 0;
    ball_control_reset_control();

    uint8_t addr = g_hardware_calibration.stepper_addr;
    uint32_t command_gap_ms =
        g_hardware_calibration.stepper_command_gap_ms;
    Emm_V5_Stop_Now(addr, false);
    vTaskDelay(pdMS_TO_TICKS(command_gap_ms));
    Emm_V5_Reset_Clog_Pro(addr);
    vTaskDelay(pdMS_TO_TICKS(command_gap_ms));
    Emm_V5_En_Control(addr, true, false);
    vTaskDelay(pdMS_TO_TICKS(command_gap_ms));
    Emm_V5_Set_QPos_Params(addr,
                           g_ball_params->motor_vel_rpm,
                           g_ball_params->motor_acc,
                           g_ball_params->motor_raf,
                           false);
    vTaskDelay(pdMS_TO_TICKS(command_gap_ms));
    ball_control_motor_level();
}

void ball_control_prepare_level(void)
{
    ball_control_prepare_level_for_menu(3U);
}

static void ball_control_start_for_menu(uint8_t menu_task)
{
    ball_control_prepare_level_for_menu(menu_task);
    g_ball_control.state = BALL_CONTROL_HOMING;
}

void ball_control_start(void)
{
    ball_control_start_for_menu(3U);
}

void ball_control_stop(void)
{
    g_ball_control_lap_mode = 0;
    g_ball_control.state        = BALL_CONTROL_IDLE;
    g_ball_control.timer_ms     = 0;
    g_ball_control.stable_ms    = 0;
    g_ball_control.reached      = false;
    g_ball_control_external_tilt_ff = 0;
    ball_control_motor_level();
}

void ball_control_emergency_stop(void)
{
    g_ball_control_lap_mode = 0;
    g_ball_control.state          = BALL_CONTROL_IDLE;
    g_ball_control.timer_ms       = 0;
    g_ball_control.stable_ms      = 0;
    g_ball_control.reached        = false;
    g_ball_control.motor_target   = 0;
    g_ball_control.desired_vel    = 0;
    g_ball_control.velocity_error = 0;
    g_ball_control_external_tilt_ff = 0;
    Emm_V5_Stop_Now(g_hardware_calibration.stepper_addr, false);
}

void ball_control_set_target(int32_t target_x)
{
    g_ball_control.target_x = target_x;
    g_ball_control.stable_ms = 0;
    g_ball_control.pos_kick_ms = 0;
    g_ball_control.pos_kick_cooldown_ms = 0;
    g_ball_control.reached = false;
}

bool ball_control_step(uint32_t dt_ms)
{
    if (g_ball_control.state == BALL_CONTROL_IDLE) {
        return false;
    }

    g_ball_control.timer_ms   += dt_ms;
    g_ball_control.no_frame_ms += dt_ms;

    switch (g_ball_control.state) {

    case BALL_CONTROL_HOMING: {
        if (g_ball_control.timer_ms >= BALL_CONTROL_HOMING_TIME_MS) {
            ball_control_reset_control();
            g_ball_control.state    = BALL_CONTROL_TRACKING;
            g_ball_control.timer_ms = 0;
        }
        break;
    }

    case BALL_CONTROL_TRACKING: {
        maixcam_ball_t ball;
        bool ball_ok = maixcam_get_ball(&ball);

        if (!ball_ok || !ball.valid) {
            ball_control_enter_lost();
            break;
        }

        if (g_ball_control.no_frame_ms >= BALL_CONTROL_LOST_TIMEOUT_MS) {
            ball_control_enter_lost();
            break;
        }

        bool new_frame;
        if (g_ball_control.first_frame) {
            new_frame = ball.valid;
        } else {
            new_frame = ball.valid && (ball.frame_id != g_ball_control.last_frame_id);
        }

        if (!new_frame) {
            break;
        }

        bool is_first = g_ball_control.first_frame;
        g_ball_control.first_frame  = false;
        g_ball_control.last_frame_id = ball.frame_id;
        uint32_t frame_dt = g_ball_control.no_frame_ms;
        g_ball_control.no_frame_ms = 0;
        (void)is_first;

        /* ── 速度级联控制 ──
         * 外环: 期望速度 = Kp_pos × 位置误差
         * 内环: 倾角 = Kp_vel × (期望速度 - 实际速度) */

        /* 1. 使用 MaixCam 发送的 vx，并低通滤波抑制视觉噪声。 */
        int32_t ball_x = (int32_t)ball.x;
        g_ball_control.last_ball_x = ball_x;
        int32_t input_vel_limit = g_ball_params->max_input_vel_px_s;
        if (input_vel_limit < 1) input_vel_limit = 1;
        int32_t raw_ball_vel = clamp_i32(
            ball.vx_px_s, -input_vel_limit, input_vel_limit);
        int32_t ball_vel = g_ball_control.last_ball_vel;
        if (!is_first) {
            int32_t alpha = g_ball_vel_filter_alpha_milli;
            if (g_ball_control_lap_mode != 0 &&
                g_ball_params->drive_vel_alpha_milli >= 0) {
                alpha = g_ball_params->drive_vel_alpha_milli;
            }
            alpha = clamp_i32(alpha, 0, 1000);
            ball_vel += (int32_t)((int64_t)alpha *
                                  (raw_ball_vel - ball_vel) / 1000);
        } else {
            ball_vel = raw_ball_vel;
        }
        g_ball_control.last_ball_vel = ball_vel;

        /* 2. 外环: 位置误差 → 期望速度 */
        int32_t raw_error = g_ball_control.target_x - ball_x;
        int32_t tilt_sign = (g_ball_tilt_sign < 0) ? -1 : 1;
        int32_t signed_error = raw_error * tilt_sign;
        int32_t abs_error = util_abs_i32(raw_error);

        int32_t active_kp_pos = g_ball_kp_pos;
        int32_t active_kp_vel = g_ball_kp_vel;
        if (g_ball_control_lap_mode != 0) {
            if (g_ball_params->drive_kp_pos_milli > 0) {
                active_kp_pos = g_ball_params->drive_kp_pos_milli;
            }
            if (g_ball_params->drive_kp_vel_milli > 0) {
                active_kp_vel = g_ball_params->drive_kp_vel_milli;
            }
        }
        int32_t desired_vel = (int32_t)((int64_t)active_kp_pos *
                                        signed_error / 1000);
        desired_vel = clamp_i32(desired_vel,
                                -g_ball_max_desired_vel,
                                g_ball_max_desired_vel);

        /* 3. 内环: 速度误差 → 倾角 */
        int32_t signed_ball_vel = ball_vel * tilt_sign;
        int32_t vel_error = desired_vel - signed_ball_vel;
        g_ball_control.desired_vel    = desired_vel;
        g_ball_control.velocity_error = vel_error;
        int32_t output = (int32_t)((int64_t)active_kp_vel *
                                   vel_error / 1000);
        /* 球速自适应倾角限制: 球速越高, 最大倾角越小, 防止过度加速 */
        int32_t abs_ball_vel = util_abs_i32(ball_vel);
        int32_t eff_max_tilt = g_ball_max_tilt;
        int32_t reduce_start = g_ball_params->tilt_reduce_start_vel;
        int32_t reduce_span = g_ball_params->tilt_reduce_vel_span;
        int32_t min_divisor = g_ball_params->min_tilt_divisor;
        if (reduce_span < 1) reduce_span = 1;
        if (min_divisor < 1) min_divisor = 1;
        if (abs_ball_vel > reduce_start) {
            int32_t reduction = (abs_ball_vel - reduce_start) *
                                g_ball_max_tilt / reduce_span;
            eff_max_tilt = g_ball_max_tilt - reduction;
            if (eff_max_tilt < g_ball_max_tilt / min_divisor)
                eff_max_tilt = g_ball_max_tilt / min_divisor;
        }
        output = clamp_i32(output,
                           -g_ball_max_tilt_neg,
                           eff_max_tilt);

        g_ball_control.last_error = raw_error;

        /* 4. Task5行驶中缩小保持区并禁用启动踢/死区猛推，
         * 让3~10px小误差仍由线性级联控制，避免弯道极限环。 */
        bool lap_drive_mode = g_ball_control_lap_mode == TASK5_LAP_MODE_CENTER ||
                              g_ball_control_lap_mode == TASK5_LAP_MODE_SPECIFIED;
        int32_t hold_tolerance = lap_drive_mode ?
            g_ball_params->drive_hold_tolerance_px : g_ball_tolerance_px;
        if (abs_error <= hold_tolerance &&
            util_abs_i32(ball_vel) <= g_ball_reached_vel_px_s) {
            g_ball_control.pos_kick_ms = 0;
            output = ball_control_lookup_hold_tilt(g_ball_control.target_x);
            output = clamp_i32(output,
                               -g_ball_max_tilt_neg,
                               g_ball_max_tilt);
        } else if (lap_drive_mode) {
            g_ball_control.pos_kick_ms = 0;
            g_ball_control.pos_kick_cooldown_ms = 0;
        } else {
            if (g_ball_control.pos_kick_cooldown_ms > frame_dt) {
                g_ball_control.pos_kick_cooldown_ms -= frame_dt;
            } else {
                g_ball_control.pos_kick_cooldown_ms = 0;
            }

            if (g_ball_control.pos_kick_ms > 0 &&
                (signed_error <= g_ball_tolerance_px ||
                 util_abs_i32(ball_vel) >= g_ball_dead_vel_th)) {
                g_ball_control.pos_kick_ms = 0;
                g_ball_control.pos_kick_cooldown_ms = BALL_CONTROL_POS_KICK_COOLDOWN_MS;
            }

            if (g_ball_control.pos_kick_ms > 0) {
                output = g_ball_control.pos_kick_tilt;
                if (g_ball_control.pos_kick_ms > frame_dt) {
                    g_ball_control.pos_kick_ms -= frame_dt;
                } else {
                    g_ball_control.pos_kick_ms = 0;
                    g_ball_control.pos_kick_cooldown_ms = BALL_CONTROL_POS_KICK_COOLDOWN_MS;
                }
            } else if (signed_error >= g_ball_tolerance_px &&
                       util_abs_i32(ball_vel) < g_ball_dead_vel_th &&
                       g_ball_control.pos_kick_cooldown_ms == 0 &&
                       output < g_ball_pos_static_tilt) {
                /* 高X方向只给有界启动脉冲，防止持续推到视觉盲区。 */
                if (signed_error >= BALL_CONTROL_POS_KICK_FAR_ERROR) {
                    g_ball_control.pos_kick_ms = BALL_CONTROL_POS_KICK_FAR_MS;
                    g_ball_control.pos_kick_tilt = g_ball_pos_static_tilt;
                } else {
                    g_ball_control.pos_kick_ms = BALL_CONTROL_POS_KICK_NEAR_MS;
                    g_ball_control.pos_kick_tilt = BALL_CONTROL_POS_KICK_NEAR_TILT;
                }
                output = g_ball_control.pos_kick_tilt;
            } else if (util_abs_i32(output) < g_ball_dead_zone &&
                       util_abs_i32(ball_vel) < g_ball_dead_vel_th) {
                output = (signed_error > 0) ? g_ball_dead_boost
                                            : -g_ball_dead_boost;
            }
        }

        /* Task5只在接近越界时增加连续位置纠偏，中心±8px内不改变
         * 已验证的小误差阻尼特性。 */
        int32_t task5_error_boost = 0;
        if (lap_drive_mode &&
            abs_error > g_ball_params->error_boost_start_px) {
            int32_t excess = abs_error -
                             g_ball_params->error_boost_start_px;
            int32_t boost_milli = g_ball_params->error_boost_milli;
            task5_error_boost = (int32_t)(
                (int64_t)boost_milli * excess / 1000);
            if (signed_error < 0) {
                task5_error_boost = -task5_error_boost;
            }
        }

        /* 车辆运动前馈：Task4/5运行时预先抵消车体惯性。 */
        output = clamp_i32(output + g_ball_control_external_tilt_ff +
                           task5_error_boost,
                           -g_ball_max_tilt_neg,
                           eff_max_tilt);

        /* 保险二: 可调输出变化率限制，0 表示关闭。
         * 注意: 比较 raw output 与上次 raw output (去掉 motor_offset)，
         * 否则偏置会阻止输出向一侧变化。 */
        if (g_ball_output_slew_pulses > 0) {
            int32_t slew = g_ball_output_slew_pulses;
            int32_t prev_raw = util_clamp_i64_to_i32(
                (int64_t)g_ball_control.motor_target - g_ball_motor_offset);
            int32_t diff = output - prev_raw;
            if (diff > slew)  output = prev_raw + slew;
            if (diff < -slew) output = prev_raw - slew;
        }

        ball_control_motor_goto(output);

        /* 5. 到达判定同时约束位置与球速，避免高速穿越目标时误判。 */
        if (abs_error < g_ball_tolerance_px &&
            util_abs_i32(ball_vel) <= g_ball_reached_vel_px_s) {
            g_ball_control.stable_ms += frame_dt;
            if (g_ball_control.stable_ms >= g_ball_stable_time_ms) {
                g_ball_control.reached = true;
            }
        } else {
            g_ball_control.stable_ms = 0;
            g_ball_control.reached   = false;
        }
        break;
    }

    case BALL_CONTROL_LOST: {
        maixcam_ball_t ball;
        bool ball_ok = maixcam_get_ball(&ball);

        if (g_ball_control.lost_recover_tilt != 0 &&
            g_ball_control.timer_ms >= BALL_CONTROL_EDGE_RECOVER_MS) {
            g_ball_control.lost_recover_tilt = 0;
            ball_control_motor_level();
        }

        if (ball_ok && ball.valid) {
            bool new_frame = (ball.frame_id != g_ball_control.last_frame_id);

            if (new_frame) {
                ball_control_reset_control();
                g_ball_control.state    = BALL_CONTROL_TRACKING;
                g_ball_control.timer_ms = 0;
            }
        }
        break;
    }

    default:
        break;
    }

    return true;
}

ball_control_state_t ball_control_get_state(void)
{
    return g_ball_control.state;
}

bool ball_control_is_active(void)
{
    return (g_ball_control.state != BALL_CONTROL_IDLE);
}

bool ball_control_is_reached(void)
{
    return g_ball_control.reached;
}

const char *ball_control_state_name(ball_control_state_t s)
{
    switch (s) {
    case BALL_CONTROL_IDLE:     return "IDLE";
    case BALL_CONTROL_HOMING:   return "HOMING";
    case BALL_CONTROL_TRACKING: return "TRACK";
    case BALL_CONTROL_LOST:     return "LOST";
    default:             return "?";
    }
}

bool ball_control_get_status(ball_control_status_t *out)
{
    if (out == NULL) {
        return false;
    }

    maixcam_ball_t ball;
    bool ball_ok = maixcam_get_ball(&ball);

    taskENTER_CRITICAL();
    out->state          = g_ball_control.state;
    out->target_x       = g_ball_control.target_x;
    out->error          = g_ball_control.last_error;
    out->motor_target   = g_ball_control.motor_target;
    out->desired_vel    = g_ball_control.desired_vel;
    out->ball_vel       = g_ball_control.last_ball_vel;
    out->velocity_error = g_ball_control.velocity_error;
    out->frame_age_ms   = g_ball_control.no_frame_ms;
    out->reached        = g_ball_control.reached;
    taskEXIT_CRITICAL();

    out->ball_x         = ball_ok ? (int32_t)ball.x : -1;
    out->integral_milli = 0;
    out->derr           = 0;
    out->ball_valid     = ball_ok && ball.valid &&
                          out->frame_age_ms < BALL_CONTROL_LOST_TIMEOUT_MS;

    return true;
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  官方任务3 — O→+5cm→-5cm
 *
 *  控制底层完全复用 ball_control_*，本层只负责：
 *    1. 启动管球内核并设定目标位置；
 *    2. 用自身像素阈值判断到达（不依赖内核 reached 标志）；
 *    3. 到达 +5cm 后停留指定时间, 再切换到 -5cm
 *    4. 记录总时间
 *
 *  状态流: IDLE → GO_POS5 → HOLD_POS5 → GO_NEG5 → COMPLETE
 *
 *  ── 参数调节 ──
 *  本任务所有时序、目标坐标和管球参数均位于 g_task3_params。
 *  修改 g_task3_params.ball 不会影响菜单Task4/5/6。
 * ═══════════════════════════════════════════════════════════════════════════ */

/* 菜单Task3独立参数别名；默认值和详细中文说明见 task_params.c/.h。 */
#define g_t3_target_pos5             (g_task3_params.target_pos5_x)
#define g_t3_target_neg5             (g_task3_params.target_neg5_x)
#define g_t3_arrive_threshold_px     (g_task3_params.arrive_threshold_px)
#define g_t3_arrive_stable_ms        (g_task3_params.arrive_stable_ms)
#define g_t3_hold_pos5_ms            (g_task3_params.hold_pos5_ms)
#define g_t3_pos_brake_margin_px     (g_task3_params.pos_brake_margin_px)
#define g_t3_pos_brake_speed_factor  (g_task3_params.pos_brake_speed_factor)
#define TASK3_POS_ARRIVE_STABLE_MS   (g_task3_params.pos_arrive_stable_ms)
#define TASK3_FINAL_MAX_VEL_PX_S     (g_task3_params.final_max_vel_px_s)

/* ── 内部状态 ── */
static task3_state_t g_t3_state       = TASK3_IDLE;
static uint32_t      g_t3_start_tick  = 0;
static uint32_t      g_t3_elapsed_ms  = 0;
static uint32_t      g_t3_stable_ms   = 0;   /* 球在到达阈值内的累计时间 */
static uint32_t      g_t3_hold_ms     = 0;   /* +5cm 停留计时 */
static int32_t       g_t3_cur_target  = 0;   /* 当前段目标 X (像素) */
static int32_t       g_t3_ball_x      = 0;   /* 当前球位置 (调试显示) */
static int32_t       g_t3_error       = 0;   /* 当前误差 (调试显示) */

/* ── 内部辅助: 用自身阈值判断球是否到达当前目标 ── */
static bool task3_check_arrived(uint32_t dt_ms, uint32_t required_stable_ms,
                                int32_t max_speed_px_s,
                                int32_t prebrake_margin_px)
{
    ball_control_status_t st;
    if (!ball_control_get_status(&st) || !st.ball_valid) {
        g_t3_stable_ms = 0;
        return false;
    }

    g_t3_ball_x = st.ball_x;
    g_t3_error  = g_t3_cur_target - st.ball_x;
    int32_t abs_error = util_abs_i32(g_t3_error);
    int32_t abs_vel = util_abs_i32(st.ball_vel);
    /* 速度自适应预制动: 球速越快, 越早切换目标, 给制动留更多距离 */
    int32_t eff_margin = prebrake_margin_px;
    if (prebrake_margin_px >= 0 && g_t3_pos_brake_speed_factor > 0) {
        eff_margin = prebrake_margin_px +
                     abs_vel * g_t3_pos_brake_speed_factor / 100;
    }
    bool prebrake = eff_margin >= 0 &&
                    g_t3_error <= eff_margin;
    bool speed_ok = max_speed_px_s < 0 ||
                    util_abs_i32(st.ball_vel) <= max_speed_px_s;

    if (prebrake) {
        g_t3_stable_ms = 0;
        return true;
    }
    if (abs_error < g_t3_arrive_threshold_px && speed_ok) {
        g_t3_stable_ms += dt_ms;
        if (g_t3_stable_ms >= required_stable_ms) {
            return true;
        }
    } else {
        g_t3_stable_ms = 0;
    }
    return false;
}

/* ── 公开接口 ── */

void task3_start(void)
{
    ball_control_start();                        /* 启动底层控制 */
    g_t3_cur_target = g_t3_target_pos5;
    ball_control_set_target(g_t3_cur_target);    /* 第一个目标: +5cm */
    g_t3_state      = TASK3_GO_POS5;
    g_t3_start_tick = xTaskGetTickCount();
    g_t3_elapsed_ms = 0;
    g_t3_stable_ms  = 0;
    g_t3_hold_ms    = 0;
    g_t3_ball_x     = 0;
    g_t3_error      = 0;
}

void task3_stop(void)
{
    ball_control_stop();
    g_t3_state = TASK3_IDLE;
}

bool task3_step(uint32_t dt_ms)
{
    if (g_t3_state == TASK3_IDLE)
        return false;

    /* 底层管球控制始终由 ball_control_* 执行。 */
    ball_control_step(dt_ms);

    switch (g_t3_state) {

    case TASK3_GO_POS5:
        /* 用自身像素阈值判断到达 +5cm */
        if (task3_check_arrived(dt_ms, TASK3_POS_ARRIVE_STABLE_MS,
                                -1, g_t3_pos_brake_margin_px)) {
            g_t3_hold_ms   = 0;
            g_t3_stable_ms = 0;
            g_t3_state     = TASK3_HOLD_POS5;
        }
        break;

    case TASK3_HOLD_POS5:
        /* 在+5cm停留，ball_control_*继续保持钢球位置。 */
        g_t3_hold_ms += dt_ms;
        if (g_t3_hold_ms >= g_t3_hold_pos5_ms) {
            /* 停留结束 → 切换目标到 -5cm */
            g_t3_cur_target = g_t3_target_neg5;
            ball_control_set_target(g_t3_cur_target);
            g_t3_stable_ms = 0;
            g_t3_state     = TASK3_GO_NEG5;
        }
        break;

    case TASK3_GO_NEG5:
        /* 用自身像素阈值判断到达 -5cm */
        if (task3_check_arrived(dt_ms, g_t3_arrive_stable_ms,
                                TASK3_FINAL_MAX_VEL_PX_S, -1)) {
            g_t3_elapsed_ms = (uint32_t)(xTaskGetTickCount() - g_t3_start_tick)
                              * portTICK_PERIOD_MS;
            g_t3_state = TASK3_COMPLETE;
        }
        break;

    case TASK3_COMPLETE:
        break;

    default:
        g_t3_state = TASK3_IDLE;
        break;
    }
    return true;
}

bool task3_is_active(void)
{
    return (g_t3_state != TASK3_IDLE && g_t3_state != TASK3_COMPLETE);
}

bool task3_get_status(task3_status_t *out)
{
    if (!out) return false;
    out->state      = g_t3_state;
    out->elapsed_ms = (g_t3_state != TASK3_IDLE &&
                       g_t3_state != TASK3_COMPLETE)
                      ? (uint32_t)(xTaskGetTickCount() - g_t3_start_tick) *
                        portTICK_PERIOD_MS
                      : g_t3_elapsed_ms;
    out->target_x   = (g_t3_state == TASK3_GO_POS5 || g_t3_state == TASK3_HOLD_POS5)
                      ? g_t3_target_pos5
                      : (g_t3_state == TASK3_GO_NEG5) ? g_t3_target_neg5
                      : 0;
    out->reached    = (g_t3_state == TASK3_COMPLETE);
    out->hold_ms    = g_t3_hold_ms;
    out->stable_ms  = g_t3_stable_ms;
    out->ball_x     = g_t3_ball_x;
    out->error      = g_t3_error;
    return true;
}

const char *task3_state_name(task3_state_t s)
{
    switch (s) {
    case TASK3_IDLE:      return "IDLE";
    case TASK3_GO_POS5:   return "+5";
    case TASK3_HOLD_POS5: return "HOLD";
    case TASK3_GO_NEG5:   return "-5";
    case TASK3_COMPLETE:  return "DONE";
    default:              return "???";
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  官方任务4 — A到B定距循线 + 钢珠中心平衡
 *
 *  状态流: IDLE → PREPARE → READY → TRACKING → BRAKING → HOLDING → COMPLETE/FAULT
 *    PREPARE: 球居中, 车轮ESTOP
 *    READY:   球已稳定在中心, 等待PB21确认
 *    TRACKING: 车辆行驶
 *    BRAKING:  到达距离, 制动
 *    HOLDING:  制动完成, 球回中确认
 *
 *  进入页面自动开始 PREPARE；READY 后按 PB21 才发车。
 *
 *  ── 参数调节 ──
 *  g_task4_params             → 距离、速度、时序和车辆前馈
 *  g_task4_params.ball        → 本任务独立管球闭环
 *  g_task4_params.line        → 本任务独立循线参数
 *  g_task4_params.speed       → 本任务独立轮速PID和编码器CPR
 * ═══════════════════════════════════════════════════════════════════════════ */

/* 菜单Task4独立参数别名。 */
#define g_t4_target_counts  (g_task4_params.target_counts)
#define g_t4_base_rpm       (g_task4_params.base_rpm)
#define g_t4_ball_target_x  (g_task4_params.ball_target_x)
#define g_t4_timeout_ms     (g_task4_params.timeout_ms)
#define g_t4_start_ff       (g_task4_params.start_ff)
#define g_t4_end_ff         (g_task4_params.end_ff)

static task4_state_t g_t4_state = TASK4_IDLE;
static task4_state_t g_t4_terminal_state = TASK4_IDLE;
static task4_fault_t g_t4_fault = TASK4_FAULT_NONE;
static uint32_t g_t4_start_tick = 0;
static uint32_t g_t4_elapsed_ms = 0;
static int32_t g_t4_max_abs_ball_error = 0;
static uint32_t g_t4_ball_violation_ms = 0;
static uint32_t g_t4_ball_invalid_ms = 0;
static uint32_t g_t4_ball_invalid_consecutive_ms = 0;
static uint32_t g_t4_prepare_start_tick = 0;
static uint32_t g_t4_prepare_stable_ms = 0;
static uint32_t g_t4_brake_hold_ms = 0;
static uint32_t g_t4_brake_stable_ms = 0;
static uint32_t g_t4_hold_elapsed_ms = 0;
static uint32_t g_t4_hold_stable_ms = 0;
static bool g_t4_emergency_stop_pending = false;

#define TASK4_BALL_LOST_STOP_MS          (g_task4_params.ball_lost_stop_ms)
#define TASK4_PREPARE_STABLE_MS          (g_task4_params.prepare_stable_ms)
#define TASK4_PREPARE_TIMEOUT_MS         (g_task4_params.prepare_timeout_ms)
#define TASK4_BRAKE_STABLE_MS            (g_task4_params.brake_stable_ms)
#define TASK4_BRAKE_TIMEOUT_MS           (g_task4_params.brake_timeout_ms)
#define TASK4_HOLD_STABLE_MS             (g_task4_params.hold_stable_ms)
#define TASK4_HOLD_TIMEOUT_MS            (g_task4_params.hold_timeout_ms)
#define TASK4_START_FF_DURATION_MS       (g_task4_params.start_ff_duration_ms)
#define TASK4_END_FF_DISTANCE_COUNTS     (g_task4_params.end_ff_distance_counts)
#define TASK4_END_FF_RAMP_DISTANCE_COUNTS (g_task4_params.end_ff_ramp_counts)

static void task4_update_ball_feedforward(
    const line_tracking_service_state_t *line)
{
    int32_t ff = 0;
    if (g_t4_state == TASK4_BRAKING) {
        ff = g_t4_end_ff;
    } else if (g_t4_state == TASK4_TRACKING) {
        if (g_t4_elapsed_ms < TASK4_START_FF_DURATION_MS) {
            ff = g_t4_start_ff;
        } else if (line != NULL &&
                   (line->reached ||
                    (line->remaining_counts > 0 &&
                     line->remaining_counts <=
                         (TASK4_END_FF_DISTANCE_COUNTS +
                          TASK4_END_FF_RAMP_DISTANCE_COUNTS)))) {
            /* end_ff按距离线性渐增：远端小补偿、近端全补偿 */
            int32_t ramp_zone = TASK4_END_FF_RAMP_DISTANCE_COUNTS;
            if (ramp_zone < 1) {
                ramp_zone = 1;
            }
            int32_t dist_to_brake = line->remaining_counts -
                TASK4_END_FF_DISTANCE_COUNTS;
            if (dist_to_brake < 0) {
                dist_to_brake = 0;
            }
            if (dist_to_brake >= ramp_zone) {
                ff = 0;
            } else {
                ff = (int32_t)((int64_t)g_t4_end_ff *
                               (ramp_zone - dist_to_brake) /
                               ramp_zone);
            }
            if (line->reached) {
                ff = g_t4_end_ff;
            }
        }
    }
    g_ball_control_external_tilt_ff = clamp_i32(ff,
                                         -g_ball_max_tilt_neg,
                                         g_ball_max_tilt);
}

static void task4_record_ball_sample(bool ball_ok,
                                     const ball_control_status_t *ball,
                                     uint32_t dt_ms)
{
    if (ball_ok && ball != NULL) {
        int32_t abs_error = util_abs_i32(ball->error);
        g_t4_ball_invalid_consecutive_ms = 0;
        if (abs_error > g_t4_max_abs_ball_error) {
            g_t4_max_abs_ball_error = abs_error;
        }
        if (abs_error > g_task4_params.ball.tolerance_px) {
            g_t4_ball_violation_ms += dt_ms;
        }
    } else {
        g_t4_ball_invalid_ms += dt_ms;
        g_t4_ball_invalid_consecutive_ms += dt_ms;
    }
}

static void task4_begin_braking(task4_state_t terminal_state,
                                task4_fault_t fault,
                                bool emergency)
{
    if (line_tracking_service_is_active_snapshot()) {
        (void)line_tracking_service_abort();
    }
    if (emergency) {
        speed_service_emergency_stop();
    } else {
        (void)speed_service_set_target(0, 0);
    }
    g_t4_terminal_state = terminal_state;
    g_t4_fault = fault;
    g_t4_brake_hold_ms = 0;
    g_t4_brake_stable_ms = 0;
    g_t4_state = TASK4_BRAKING;
}

bool task4_prepare(void)
{
    if (task4_is_active() || line_tracking_service_is_active_snapshot()) {
        return false;
    }

    speed_service_emergency_stop();
    if (!speed_service_select_profile(SPEED_PROFILE_TASK4_COMBINED) ||
        !line_tracking_service_select_profile(
            LINE_TRACKING_PROFILE_TASK4_COMBINED)) {
        g_t4_state = TASK4_FAULT;
        g_t4_fault = TASK4_FAULT_START;
        return false;
    }
    ball_control_start_for_menu(4U);
    ball_control_set_target(g_t4_ball_target_x);
    g_t4_state = TASK4_PREPARE;
    g_t4_terminal_state = TASK4_IDLE;
    g_t4_fault = TASK4_FAULT_NONE;
    g_t4_start_tick = 0;
    g_t4_prepare_start_tick = xTaskGetTickCount();
    g_t4_prepare_stable_ms = 0;
    g_t4_elapsed_ms = 0;
    g_t4_max_abs_ball_error = 0;
    g_t4_ball_violation_ms = 0;
    g_t4_ball_invalid_ms = 0;
    g_t4_ball_invalid_consecutive_ms = 0;
    g_t4_brake_hold_ms = 0;
    g_t4_brake_stable_ms = 0;
    g_t4_hold_elapsed_ms = 0;
    g_t4_hold_stable_ms = 0;
    g_ball_control_external_tilt_ff = 0;
    g_t4_emergency_stop_pending = false;
    return true;
}

bool task4_start(void)
{
    return task4_prepare();
}

bool task4_is_ready(void)
{
    return g_t4_state == TASK4_READY;
}

bool task4_start_motion(void)
{
    ball_control_status_t ball = {0};
    if (g_t4_state != TASK4_READY ||
        line_tracking_service_is_active_snapshot() ||
        !ball_control_get_status(&ball) || !ball.ball_valid ||
        util_abs_i32(ball.error) > g_task4_params.ball.tolerance_px) {
        return false;
    }

    speed_service_clear_emergency_stop();
    if (!line_tracking_service_start(g_t4_target_counts, g_t4_base_rpm)) {
        speed_service_emergency_stop();
        g_t4_fault = TASK4_FAULT_START;
        g_t4_state = TASK4_FAULT;
        return false;
    }
    g_t4_start_tick = xTaskGetTickCount();
    g_t4_elapsed_ms = 0;
    g_t4_state = TASK4_TRACKING;
    return true;
}

void task4_stop(void)
{
    if (task4_is_active()) {
        task4_begin_braking(TASK4_IDLE, TASK4_FAULT_NONE, true);
    } else {
        speed_service_emergency_stop();
        ball_control_stop();
        g_t4_state = TASK4_IDLE;
    }
}

void task4_emergency_stop(void)
{
    bool was_active = task4_is_active();
    if (line_tracking_service_is_active_snapshot()) {
        (void)line_tracking_service_abort();
    }
    speed_service_emergency_stop();
    ball_control_emergency_stop();
    g_t4_terminal_state = TASK4_IDLE;
    g_t4_fault = TASK4_FAULT_NONE;
    g_t4_brake_hold_ms = 0;
    g_t4_brake_stable_ms = 0;
    g_t4_hold_elapsed_ms = 0;
    g_t4_hold_stable_ms = 0;
    g_ball_control_external_tilt_ff = 0;
    g_t4_emergency_stop_pending = was_active;
    g_t4_state = was_active ? TASK4_BRAKING : TASK4_IDLE;
}

bool task4_step(uint32_t dt_ms)
{
    if (!task4_is_active()) {
        return false;
    }

    if (g_t4_state == TASK4_PREPARE || g_t4_state == TASK4_READY) {
        if (ball_control_is_active()) {
            ball_control_step(dt_ms);
        }
        speed_service_step_10ms();

        if (g_t4_state == TASK4_READY) {
            ball_control_status_t ready_ball = {0};
            if (!ball_control_get_status(&ready_ball) || !ready_ball.ball_valid ||
                util_abs_i32(ready_ball.error) >
                    g_task4_params.ball.tolerance_px) {
                g_t4_state = TASK4_PREPARE;
                g_t4_prepare_start_tick = xTaskGetTickCount();
                g_t4_prepare_stable_ms = 0;
            }
            return true;
        }

        if (g_t4_state == TASK4_PREPARE) {
            ball_control_status_t prepare_ball = {0};
            bool prepare_ball_ok = ball_control_get_status(&prepare_ball) &&
                                   prepare_ball.ball_valid;
            uint32_t prepare_ms =
                (uint32_t)(xTaskGetTickCount() - g_t4_prepare_start_tick) *
                portTICK_PERIOD_MS;
            if (prepare_ball_ok && util_abs_i32(prepare_ball.error) <=
                                   g_task4_params.ball.tolerance_px) {
                g_t4_prepare_stable_ms += dt_ms;
            } else {
                g_t4_prepare_stable_ms = 0;
            }

            if (g_t4_prepare_stable_ms >= TASK4_PREPARE_STABLE_MS) {
                g_t4_state = TASK4_READY;
            } else if (prepare_ms > TASK4_PREPARE_TIMEOUT_MS) {
                speed_service_emergency_stop();
                g_t4_fault = TASK4_FAULT_BALL_LOST;
                g_t4_state = TASK4_FAULT;
            }
        }
        return true;
    }

    line_tracking_service_step_10ms();
    line_tracking_service_state_t line = {0};
    bool line_ok = line_tracking_service_get_state(&line);
    task4_update_ball_feedforward(line_ok ? &line : NULL);
    if (ball_control_is_active()) {
        ball_control_step(dt_ms);
    }
    speed_service_step_10ms();

    ball_control_status_t ball = {0};
    bool ball_ok = ball_control_get_status(&ball) && ball.ball_valid;

    if (g_t4_state == TASK4_TRACKING) {
        g_t4_elapsed_ms = (uint32_t)(xTaskGetTickCount() - g_t4_start_tick) *
                          portTICK_PERIOD_MS;

        task4_record_ball_sample(ball_ok, &ball, dt_ms);

        if (!line_ok || line.fault != LINE_TRACK_FAULT_NONE) {
            task4_begin_braking(TASK4_FAULT, TASK4_FAULT_LINE, true);
        } else if (g_t4_ball_invalid_consecutive_ms >=
                   TASK4_BALL_LOST_STOP_MS) {
            task4_begin_braking(TASK4_FAULT,
                                TASK4_FAULT_BALL_LOST, true);
        } else if (line.reached) {
            if (g_t4_elapsed_ms <= g_t4_timeout_ms) {
                task4_begin_braking(TASK4_COMPLETE,
                                    TASK4_FAULT_NONE, false);
            } else {
                task4_begin_braking(TASK4_FAULT,
                                    TASK4_FAULT_TIMEOUT, true);
            }
        } else if (g_t4_elapsed_ms > g_t4_timeout_ms) {
            task4_begin_braking(TASK4_FAULT,
                                TASK4_FAULT_TIMEOUT, true);
        } else if (!line.active) {
            task4_begin_braking(TASK4_FAULT, TASK4_FAULT_LINE, true);
        }
    }

    if (g_t4_state == TASK4_BRAKING) {
        speed_service_state_t wheel = {0};
        bool wheel_ok = speed_service_get_state(&wheel);
        if (g_t4_brake_hold_ms > 0U) {
            task4_record_ball_sample(ball_ok, &ball, dt_ms);
        }
        g_t4_brake_hold_ms += dt_ms;

        if (wheel_ok && util_abs_i32(wheel.left_rpm) <=
                            g_task4_params.wheel_stop_rpm &&
            util_abs_i32(wheel.right_rpm) <=
                g_task4_params.wheel_stop_rpm) {
            g_t4_brake_stable_ms += dt_ms;
        } else {
            g_t4_brake_stable_ms = 0;
        }

        if (g_t4_brake_stable_ms >= TASK4_BRAKE_STABLE_MS &&
            !line_tracking_service_is_active_snapshot()) {
            if (g_t4_terminal_state == TASK4_COMPLETE) {
                g_ball_control_external_tilt_ff = 0;
                g_t4_hold_elapsed_ms = 0;
                g_t4_hold_stable_ms = 0;
                g_t4_state = TASK4_HOLDING;
            } else {
                g_t4_state = g_t4_terminal_state;
                g_ball_control_external_tilt_ff = 0;
                if (g_t4_state == TASK4_IDLE &&
                    !g_t4_emergency_stop_pending) {
                    ball_control_stop();
                }
                g_t4_emergency_stop_pending = false;
            }
        } else if (g_t4_brake_hold_ms > TASK4_BRAKE_TIMEOUT_MS) {
            speed_service_emergency_stop();
            g_ball_control_external_tilt_ff = 0;
            if (g_t4_terminal_state == TASK4_COMPLETE) {
                g_t4_fault = TASK4_FAULT_LINE;
                g_t4_state = TASK4_FAULT;
            } else {
                g_t4_state = g_t4_terminal_state;
            }
        }
    } else if (g_t4_state == TASK4_HOLDING) {
        g_t4_hold_elapsed_ms += dt_ms;
        task4_record_ball_sample(ball_ok, &ball, dt_ms);
        if (ball_ok && util_abs_i32(ball.error) <=
                       g_task4_params.ball.tolerance_px) {
            g_t4_hold_stable_ms += dt_ms;
        } else {
            g_t4_hold_stable_ms = 0;
        }

        if (g_t4_hold_stable_ms >= TASK4_HOLD_STABLE_MS) {
            g_t4_state = TASK4_COMPLETE;
        } else if (g_t4_ball_invalid_consecutive_ms >=
                       TASK4_BALL_LOST_STOP_MS ||
                   g_t4_hold_elapsed_ms > TASK4_HOLD_TIMEOUT_MS) {
            g_t4_fault = TASK4_FAULT_BALL_LOST;
            g_t4_state = TASK4_FAULT;
        }
    }
    return true;
}

bool task4_is_active(void)
{
    return g_t4_state == TASK4_PREPARE ||
           g_t4_state == TASK4_READY ||
           g_t4_state == TASK4_TRACKING ||
           g_t4_state == TASK4_BRAKING ||
           g_t4_state == TASK4_HOLDING;
}

bool task4_get_status(task4_status_t *out)
{
    if (out == NULL) {
        return false;
    }

    line_tracking_service_state_t line = {0};
    speed_service_state_t speed = {0};
    ball_control_status_t ball = {0};
    bool line_ok = line_tracking_service_get_state(&line);
    bool speed_ok = speed_service_get_state(&speed);
    bool ball_ok = ball_control_get_status(&ball);

    out->state = g_t4_state;
    out->fault = g_t4_fault;
    out->elapsed_ms = g_t4_elapsed_ms;
    out->target_counts = g_t4_target_counts;
    out->travelled_counts = line_ok ? line.travelled_counts : 0;
    out->remaining_counts = line_ok ? line.remaining_counts : 0;
    out->line_position = line_ok ? line.line_position : 0;
    out->turn_rpm = line_ok ? line.turn_rpm : 0;
    out->left_rpm = speed_ok ? speed.left_rpm : 0;
    out->right_rpm = speed_ok ? speed.right_rpm : 0;
    out->ball_x = ball_ok ? ball.ball_x : -1;
    out->ball_error = ball_ok ? ball.error : 0;
    out->max_abs_ball_error = g_t4_max_abs_ball_error;
    out->ball_accel_ff = g_ball_control_external_tilt_ff;
    out->ball_violation_ms = g_t4_ball_violation_ms;
    out->ball_invalid_ms = g_t4_ball_invalid_ms;
    out->line_fault = line_ok ? (uint8_t)line.fault :
                               (uint8_t)LINE_TRACK_FAULT_SPEED_STATE;
    out->ball_valid = ball_ok && ball.ball_valid;
    out->reached = g_t4_state == TASK4_COMPLETE;
    return true;
}

const char *task4_state_name(task4_state_t state)
{
    switch (state) {
    case TASK4_IDLE:     return "IDLE";
    case TASK4_PREPARE:  return "CENTER";
    case TASK4_READY:    return "READY";
    case TASK4_TRACKING: return "TRACK";
    case TASK4_HOLDING:  return "HOLD";
    case TASK4_BRAKING:  return "BRAKE";
    case TASK4_COMPLETE: return "DONE";
    case TASK4_FAULT:    return "FAULT";
    default:             return "???";
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task5: 完整一圈循线 + 钢珠中心平衡 (H题第五问)
 *
 *  终点只由A点横向启停线决定：启动时先确认传感器已经驶离起点黑线，
 *  此后第一次重新稳定检测到检查点就停车。编码器不参与终点判断。
 *
 *  菜单Task5全部参数位于 g_task5_params；菜单Task6全部参数位于
 *  g_task6_params（其一圈参数在 .lap 内）。两者只复用状态机代码，
 *  ball/line/speed/时序/前馈/保持表均为独立存储。
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Task5/6共用状态机，但参数块完全独立。ball/line/speed控制参数保持
 * Watch在线可调；状态机顶层参数在按键启动时统一锁存，避免运行中只让
 * 一半逻辑读到新值。active_params只用于选择任务参数和写回圈长学习值。 */
typedef struct {
    int32_t base_rpm;
    uint32_t timeout_ms;
    int32_t start_ff;
    int32_t brake_ff;
    int32_t corner_ff_milli;
    int32_t lap_estimate_counts;
    int32_t decel_counts;
    int32_t end_min_rpm;
    uint32_t ball_lost_stop_ms;
    uint32_t prepare_stable_ms;
    uint32_t prepare_timeout_ms;
    uint32_t checkpoint_clear_ms;
    uint32_t checkpoint_confirm_ms;
    uint32_t start_ff_duration_ms;
    int32_t corner_ff_limit;
    int32_t corner_ff_attack_per_tick;
    int32_t corner_ff_release_per_tick;
    int32_t approach_ff_limit;
    int32_t final_corner_ff_percent;
    int32_t curve1_start_percent;
    int32_t curve1_release_percent;
    int32_t curve1_end_percent;
    int32_t curve2_start_percent;
    int32_t curve_entry_turn_rpm;
    int32_t drive_ff_limit;
    int32_t brake_start_min_rpm;
    uint32_t brake_ramp_ms;
    uint32_t brake_stable_ms;
    uint32_t brake_timeout_ms;
    uint32_t hold_stable_ms;
    uint32_t hold_timeout_ms;
    int32_t wheel_stop_rpm;
    int32_t decel_min_counts;
    int32_t lap_learn_min_counts;
    int32_t lap_learn_max_counts;
} task5_run_params_t;

static volatile lap_ball_params_t *g_t5_active_params = &g_task5_params;
static task5_run_params_t g_t5_run_params;
static task5_state_t g_t5_state = TASK5_IDLE;
static task5_fault_t g_t5_fault = TASK5_FAULT_NONE;
static task5_lap_mode_t g_t5_active_mode = TASK5_LAP_MODE_CENTER;
static int32_t g_t5_active_target_x = 0;

#define g_t5_active_base_rpm            (g_t5_run_params.base_rpm)
#define g_t5_active_timeout_ms          (g_t5_run_params.timeout_ms)
#define g_t5_active_start_ff            (g_t5_run_params.start_ff)
#define g_t5_active_brake_ff            (g_t5_run_params.brake_ff)
#define g_t5_active_corner_ff           (g_t5_run_params.corner_ff_milli)
#define g_t5_active_lap_estimate_counts (g_t5_run_params.lap_estimate_counts)
#define g_t5_active_decel_counts        (g_t5_run_params.decel_counts)
#define g_t5_active_end_min_rpm         (g_t5_run_params.end_min_rpm)
static uint32_t g_t5_start_tick = 0;
static uint32_t g_t5_motion_start_tick = 0;
static uint32_t g_t5_elapsed_ms = 0;
static uint32_t g_t5_prepare_stable_ms = 0;
static uint32_t g_t5_checkpoint_clear_ms = 0;
static uint32_t g_t5_checkpoint_detect_ms = 0;
static uint32_t g_t5_brake_elapsed_ms = 0;
static uint32_t g_t5_brake_stable_ms = 0;
static uint32_t g_t5_hold_elapsed_ms = 0;
static uint32_t g_t5_hold_stable_ms = 0;
static uint32_t g_t5_ball_violation_ms = 0;
static uint32_t g_t5_ball_invalid_ms = 0;
static uint32_t g_t5_ball_invalid_consecutive_ms = 0;
static int32_t g_t5_max_abs_ball_error = 0;
static int32_t g_t5_brake_start_rpm = 0;
static int32_t g_t5_travelled_counts = 0;
static int32_t g_t5_approach_rpm = 0;
static int32_t g_t5_corner_ff_filtered = 0;
static bool g_t5_curve_active = false;
static uint8_t g_t5_mask = 0xFF;
static uint8_t g_t5_strong = 0;
static uint8_t g_t5_checkpoint_count = 0;
static uint8_t g_t5_checkpoint_arm_count = 0;
static bool g_t5_checkpoint_detected = false;
static bool g_t5_checkpoint_armed = false;
static bool g_t5_service_abort_pending = false;

#define TASK5_BALL_LOST_STOP_MS       (g_t5_run_params.ball_lost_stop_ms)
#define TASK5_PREPARE_STABLE_MS       (g_t5_run_params.prepare_stable_ms)
#define TASK5_PREPARE_TIMEOUT_MS      (g_t5_run_params.prepare_timeout_ms)
#define TASK5_CHECKPOINT_CLEAR_MS     (g_t5_run_params.checkpoint_clear_ms)
#define TASK5_CHECKPOINT_CONFIRM_MS   (g_t5_run_params.checkpoint_confirm_ms)
#define TASK5_START_FF_DURATION_MS    (g_t5_run_params.start_ff_duration_ms)
#define TASK5_CORNER_FF_LIMIT         (g_t5_run_params.corner_ff_limit)
#define TASK5_CORNER_FF_ATTACK_PER_TICK (g_t5_run_params.corner_ff_attack_per_tick)
#define TASK5_CORNER_FF_RELEASE_PER_TICK (g_t5_run_params.corner_ff_release_per_tick)
#define TASK5_APPROACH_FF_LIMIT       (g_t5_run_params.approach_ff_limit)
#define TASK5_FINAL_CORNER_FF_PERCENT (g_t5_run_params.final_corner_ff_percent)
#define TASK5_CURVE1_START_PERCENT    (g_t5_run_params.curve1_start_percent)
#define TASK5_CURVE1_RELEASE_START_PERCENT (g_t5_run_params.curve1_release_percent)
#define TASK5_CURVE1_END_PERCENT      (g_t5_run_params.curve1_end_percent)
#define TASK5_CURVE2_START_PERCENT    (g_t5_run_params.curve2_start_percent)
#define TASK5_CURVE_ENTRY_TURN_RPM    (g_t5_run_params.curve_entry_turn_rpm)
#define TASK5_DRIVE_FF_LIMIT          (g_t5_run_params.drive_ff_limit)
#define TASK5_BRAKE_RAMP_MS           (g_t5_run_params.brake_ramp_ms)
#define TASK5_BRAKE_STABLE_MS         (g_t5_run_params.brake_stable_ms)
#define TASK5_BRAKE_TIMEOUT_MS        (g_t5_run_params.brake_timeout_ms)
#define TASK5_HOLD_STABLE_MS          (g_t5_run_params.hold_stable_ms)
#define TASK5_HOLD_TIMEOUT_MS         (g_t5_run_params.hold_timeout_ms)

static void task5_latch_run_params(volatile lap_ball_params_t *source)
{
    g_t5_run_params.base_rpm = source->base_rpm;
    g_t5_run_params.timeout_ms = source->timeout_ms;
    g_t5_run_params.start_ff = source->start_ff;
    g_t5_run_params.brake_ff = source->brake_ff;
    g_t5_run_params.corner_ff_milli = source->corner_ff_milli;
    g_t5_run_params.lap_estimate_counts = source->lap_estimate_counts;
    g_t5_run_params.decel_counts = source->decel_counts;
    g_t5_run_params.end_min_rpm = source->end_min_rpm;
    g_t5_run_params.ball_lost_stop_ms = source->ball_lost_stop_ms;
    g_t5_run_params.prepare_stable_ms = source->prepare_stable_ms;
    g_t5_run_params.prepare_timeout_ms = source->prepare_timeout_ms;
    g_t5_run_params.checkpoint_clear_ms = source->checkpoint_clear_ms;
    g_t5_run_params.checkpoint_confirm_ms = source->checkpoint_confirm_ms;
    g_t5_run_params.start_ff_duration_ms = source->start_ff_duration_ms;
    g_t5_run_params.corner_ff_limit = source->corner_ff_limit;
    g_t5_run_params.corner_ff_attack_per_tick =
        source->corner_ff_attack_per_tick;
    g_t5_run_params.corner_ff_release_per_tick =
        source->corner_ff_release_per_tick;
    g_t5_run_params.approach_ff_limit = source->approach_ff_limit;
    g_t5_run_params.final_corner_ff_percent = source->final_corner_ff_percent;
    g_t5_run_params.curve1_start_percent = source->curve1_start_percent;
    g_t5_run_params.curve1_release_percent = source->curve1_release_percent;
    g_t5_run_params.curve1_end_percent = source->curve1_end_percent;
    g_t5_run_params.curve2_start_percent = source->curve2_start_percent;
    g_t5_run_params.curve_entry_turn_rpm = source->curve_entry_turn_rpm;
    g_t5_run_params.drive_ff_limit = source->drive_ff_limit;
    g_t5_run_params.brake_start_min_rpm = source->brake_start_min_rpm;
    g_t5_run_params.brake_ramp_ms = source->brake_ramp_ms;
    g_t5_run_params.brake_stable_ms = source->brake_stable_ms;
    g_t5_run_params.brake_timeout_ms = source->brake_timeout_ms;
    g_t5_run_params.hold_stable_ms = source->hold_stable_ms;
    g_t5_run_params.hold_timeout_ms = source->hold_timeout_ms;
    g_t5_run_params.wheel_stop_rpm = source->wheel_stop_rpm;
    g_t5_run_params.decel_min_counts = source->decel_min_counts;
    g_t5_run_params.lap_learn_min_counts = source->lap_learn_min_counts;
    g_t5_run_params.lap_learn_max_counts = source->lap_learn_max_counts;
}

static int32_t task5_percent_of_lap(int32_t percent)
{
    return util_clamp_i64_to_i32(
        (int64_t)g_t5_active_lap_estimate_counts * percent / 100);
}

static int32_t task5_approach_value(int32_t current,
                                    int32_t target,
                                    int32_t step)
{
    if (step < 0) step = 0;
    if (current < target) {
        int32_t next = current + step;
        return next > target ? target : next;
    }
    if (current > target) {
        int32_t next = current - step;
        return next < target ? target : next;
    }
    return current;
}

static void task5_record_ball_sample(bool ball_ok,
                                     const ball_control_status_t *ball,
                                     uint32_t dt_ms)
{
    if (ball_ok && ball != NULL) {
        int32_t abs_error = util_abs_i32(ball->error);
        g_t5_ball_invalid_consecutive_ms = 0;
        if (abs_error > g_t5_max_abs_ball_error) {
            g_t5_max_abs_ball_error = abs_error;
        }
        if (abs_error > g_ball_params->tolerance_px) {
            g_t5_ball_violation_ms += dt_ms;
        }
    } else {
        g_t5_ball_invalid_ms += dt_ms;
        g_t5_ball_invalid_consecutive_ms += dt_ms;
    }
}

static void task5_enter_fault(task5_fault_t fault)
{
    if (line_tracking_service_is_active_snapshot()) {
        g_t5_service_abort_pending = line_tracking_service_abort();
    }
    speed_service_emergency_stop();
    ball_control_emergency_stop();
    (void)line_tracking_service_set_speed_cap(0);
    g_ball_control_external_tilt_ff = 0;
    g_t5_fault = fault;
    g_t5_state = TASK5_FAULT;
}

static void task5_begin_braking(const speed_service_state_t *speed)
{
    int32_t left = speed != NULL ? util_abs_i32(speed->left_rpm) : 0;
    int32_t right = speed != NULL ? util_abs_i32(speed->right_rpm) : 0;
    g_t5_brake_start_rpm = (left + right) / 2;
    if (g_t5_brake_start_rpm <
        g_t5_run_params.brake_start_min_rpm) {
        g_t5_brake_start_rpm = g_t5_run_params.brake_start_min_rpm;
    }
    (void)line_tracking_service_abort();
    g_t5_brake_elapsed_ms = 0;
    g_t5_brake_stable_ms = 0;
    g_ball_control_external_tilt_ff = clamp_i32(
        g_t5_active_brake_ff, -g_ball_max_tilt_neg, g_ball_max_tilt);
    g_t5_state = TASK5_BRAKING;
}

static bool task5_start_common(int32_t ball_target_x,
                               task5_lap_mode_t mode)
{
    if (g_t5_service_abort_pending) {
        return false;
    }
    if (task5_is_active()) {
        /* 禁止运行中被另一菜单模式“借成功返回”后改写动态前馈。 */
        return false;
    }
    if (line_tracking_service_is_active_snapshot()) {
        g_t5_fault = TASK5_FAULT_START;
        g_t5_state = TASK5_FAULT;
        return false;
    }

    speed_service_profile_t speed_profile =
        mode == TASK5_LAP_MODE_SPECIFIED ?
        SPEED_PROFILE_TASK6_SPECIFIED_BALL : SPEED_PROFILE_TASK5_LAP_BALL;
    line_tracking_profile_t line_profile =
        mode == TASK5_LAP_MODE_SPECIFIED ?
        LINE_TRACKING_PROFILE_TASK6_SPECIFIED_BALL :
        LINE_TRACKING_PROFILE_TASK5_LAP_BALL;

    g_t5_active_mode = mode;
    g_t5_active_target_x = ball_target_x;
    g_t5_active_params = mode == TASK5_LAP_MODE_SPECIFIED ?
                         &g_task6_params.lap : &g_task5_params;
    task5_latch_run_params(g_t5_active_params);

    g_t5_start_tick = xTaskGetTickCount();
    speed_service_emergency_stop();
    if (!speed_service_select_profile(speed_profile) ||
        !line_tracking_service_select_profile(line_profile)) {
        g_t5_fault = TASK5_FAULT_START;
        g_t5_state = TASK5_FAULT;
        return false;
    }
    ball_control_start_for_menu(mode == TASK5_LAP_MODE_SPECIFIED ? 6U : 5U);
    ball_control_set_target(g_t5_active_target_x);
    g_t5_state = TASK5_PREPARE;
    g_t5_fault = TASK5_FAULT_NONE;
    g_t5_motion_start_tick = 0;
    g_t5_elapsed_ms = 0;
    g_t5_prepare_stable_ms = 0;
    g_t5_checkpoint_clear_ms = 0;
    g_t5_checkpoint_detect_ms = 0;
    g_t5_brake_elapsed_ms = 0;
    g_t5_brake_stable_ms = 0;
    g_t5_hold_elapsed_ms = 0;
    g_t5_hold_stable_ms = 0;
    g_t5_ball_violation_ms = 0;
    g_t5_ball_invalid_ms = 0;
    g_t5_ball_invalid_consecutive_ms = 0;
    g_t5_max_abs_ball_error = 0;
    g_t5_brake_start_rpm = 0;
    g_t5_travelled_counts = 0;
    g_t5_approach_rpm = g_t5_active_base_rpm;
    g_t5_corner_ff_filtered = 0;
    g_t5_curve_active = false;
    g_t5_mask = 0xFF;
    g_t5_strong = 0;
    g_t5_checkpoint_count = 0;
    g_t5_checkpoint_arm_count = 0;
    g_t5_checkpoint_detected = false;
    g_t5_checkpoint_armed = false;
    g_t5_service_abort_pending = false;
    g_ball_control_external_tilt_ff = 0;
    return true;
}

bool task5_start(void)
{
    return task5_start_common(g_task5_params.ball_target_x,
                              TASK5_LAP_MODE_CENTER);
}

bool task6_start_auto(void)
{
    maixcam_ball_t ball = {0};
    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (!maixcam_get_ball(&ball) || !ball.valid ||
        now_ms - ball.tick_ms > g_task6_params.ball_fresh_max_ms ||
        ball.x < g_task6_params.auto_target_min_x ||
        ball.x > g_task6_params.auto_target_max_x) {
        return false;
    }

    g_task6_params.lap.ball_target_x = (int32_t)ball.x;
    if (!task5_start_common(g_task6_params.lap.ball_target_x,
                            TASK5_LAP_MODE_SPECIFIED)) {
        return false;
    }

    /* 任意位置模式按Task6自己的坐标区间插值启停前馈。 */
    int32_t target_x = g_task6_params.lap.ball_target_x;
    int32_t low_x = g_task6_params.ff_low_x;
    int32_t high_x = g_task6_params.ff_high_x;
    if (high_x <= low_x) {
        high_x = (low_x < INT32_MAX) ? low_x + 1 : low_x;
    }
    if (target_x <= low_x) {
        g_t5_active_start_ff = g_task6_params.start_ff_low;
        g_t5_active_brake_ff = g_task6_params.brake_ff_low;
    } else if (target_x >= high_x) {
        g_t5_active_start_ff = g_task6_params.start_ff_high;
        g_t5_active_brake_ff = g_task6_params.brake_ff_high;
    } else {
        int64_t offset = (int64_t)target_x - low_x;
        int64_t span = (int64_t)high_x - low_x;
        g_t5_active_start_ff = util_clamp_i64_to_i32(
            (int64_t)g_task6_params.start_ff_low +
            ((int64_t)g_task6_params.start_ff_high -
             g_task6_params.start_ff_low) * offset / span);
        g_t5_active_brake_ff = util_clamp_i64_to_i32(
            (int64_t)g_task6_params.brake_ff_low +
            ((int64_t)g_task6_params.brake_ff_high -
             g_task6_params.brake_ff_low) * offset / span);
    }
    return true;
}

void task5_emergency_stop(void)
{
    if (line_tracking_service_is_active_snapshot()) {
        g_t5_service_abort_pending = line_tracking_service_abort();
    }
    speed_service_emergency_stop();
    ball_control_emergency_stop();
    (void)line_tracking_service_set_speed_cap(0);
    g_ball_control_external_tilt_ff = 0;
    g_t5_fault = TASK5_FAULT_NONE;
    g_t5_state = TASK5_IDLE;
}

void task5_stop(void)
{
    task5_emergency_stop();
}

bool task5_step(uint32_t dt_ms)
{
    if (!task5_is_active()) {
        return false;
    }

    if (g_t5_service_abort_pending) {
        line_tracking_service_step_10ms();
        speed_service_step_10ms();
        if (!line_tracking_service_is_active_snapshot()) {
            g_t5_service_abort_pending = false;
        }
        if (g_t5_state == TASK5_IDLE || g_t5_state == TASK5_FAULT) {
            return true;
        }
    }

    if (g_t5_state == TASK5_PREPARE ||
        g_t5_state == TASK5_TRACKING) {
        g_t5_elapsed_ms =
            (uint32_t)(xTaskGetTickCount() - g_t5_start_tick) *
            portTICK_PERIOD_MS;
    }

    if (g_t5_state == TASK5_PREPARE) {
        if (ball_control_is_active()) {
            ball_control_step(dt_ms);
        }
        speed_service_step_10ms();

        ball_control_status_t ball = {0};
        bool ball_ok = ball_control_get_status(&ball) && ball.ball_valid;
        if (ball_ok && util_abs_i32(ball.error) <=
                       g_ball_params->tolerance_px) {
            g_t5_prepare_stable_ms += dt_ms;
        } else {
            g_t5_prepare_stable_ms = 0;
        }

        if (g_t5_prepare_stable_ms >= TASK5_PREPARE_STABLE_MS) {
            g_ball_control_lap_mode = g_t5_active_mode;
            speed_service_clear_emergency_stop();
            if (!line_tracking_service_start_checkpoint(
                    g_t5_active_base_rpm)) {
                task5_enter_fault(TASK5_FAULT_START);
            } else {
                g_t5_motion_start_tick = xTaskGetTickCount();
                g_t5_state = TASK5_TRACKING;
            }
        } else if (g_t5_elapsed_ms > TASK5_PREPARE_TIMEOUT_MS) {
            task5_enter_fault(TASK5_FAULT_BALL_LOST);
        }
        return true;
    }

    if (g_t5_state == TASK5_TRACKING) {
        line_tracking_service_step_10ms();
        line_tracking_service_state_t line = {0};
        speed_service_state_t speed = {0};
        bool line_ok = line_tracking_service_get_state(&line);
        bool speed_ok = speed_service_get_state(&speed);
        uint32_t drive_ms =
            (uint32_t)(xTaskGetTickCount() - g_t5_motion_start_tick) *
            portTICK_PERIOD_MS;

        int32_t end_ff = 0;
        g_t5_approach_rpm = g_t5_active_base_rpm;
        if (line_ok) {
            g_t5_travelled_counts = line.travelled_counts;
            int32_t decel_counts = g_t5_active_decel_counts;
            int32_t decel_min_counts = g_t5_run_params.decel_min_counts;
            if (decel_min_counts < 2) decel_min_counts = 2;
            int32_t decel_start = util_clamp_i64_to_i32(
                (int64_t)g_t5_active_lap_estimate_counts - decel_counts);
            if (decel_counts >= decel_min_counts && decel_counts >= 2 &&
                line.travelled_counts > decel_start) {
                int32_t progress = util_clamp_i64_to_i32(
                    (int64_t)line.travelled_counts - decel_start);
                int32_t ramp_counts = decel_counts / 2;
                if (progress < 0) {
                    progress = 0;
                }
                if (progress < ramp_counts) {
                    g_t5_approach_rpm = g_t5_active_base_rpm -
                        (int32_t)((int64_t)(g_t5_active_base_rpm -
                         g_t5_active_end_min_rpm) * progress /
                        ramp_counts);
                    end_ff = (int32_t)((int64_t)g_t5_active_brake_ff *
                                       progress / ramp_counts);
                } else {
                    g_t5_approach_rpm = g_t5_active_end_min_rpm;
                    int32_t release_counts = decel_counts - ramp_counts;
                    if (progress < decel_counts && release_counts > 0) {
                        end_ff = (int32_t)((int64_t)g_t5_active_brake_ff *
                            (decel_counts - progress) / release_counts);
                    }
                }
                if (g_t5_approach_rpm < g_t5_active_end_min_rpm) {
                    g_t5_approach_rpm = g_t5_active_end_min_rpm;
                }
                end_ff = clamp_i32(end_ff,
                                   -TASK5_APPROACH_FF_LIMIT,
                                   TASK5_APPROACH_FF_LIMIT);
                (void)line_tracking_service_set_speed_cap(
                    g_t5_approach_rpm);
            } else {
                (void)line_tracking_service_set_speed_cap(0);
            }
        }

        int32_t drive_ff = drive_ms < TASK5_START_FF_DURATION_MS ?
            g_t5_active_start_ff : 0;
        if (line_ok) {
            int32_t curve1_start = task5_percent_of_lap(
                TASK5_CURVE1_START_PERCENT);
            int32_t curve1_release_start = task5_percent_of_lap(
                TASK5_CURVE1_RELEASE_START_PERCENT);
            int32_t curve1_end = task5_percent_of_lap(
                TASK5_CURVE1_END_PERCENT);
            int32_t curve2_start = task5_percent_of_lap(
                TASK5_CURVE2_START_PERCENT);
            bool curve_window =
                (line.travelled_counts >= curve1_start &&
                 line.travelled_counts <= curve1_end) ||
                line.travelled_counts >= curve2_start;
            if (!curve_window) {
                g_t5_curve_active = false;
            } else if (line.turn_rpm <= TASK5_CURVE_ENTRY_TURN_RPM) {
                g_t5_curve_active = true;
            }

            int32_t corner_base = -(g_t5_active_corner_ff / 50);
            int32_t corner_target = 0;
            if (g_t5_curve_active &&
                line.travelled_counts < curve2_start) {
                if (line.travelled_counts <= curve1_release_start) {
                    corner_target = corner_base;
                } else if (curve1_end > curve1_release_start &&
                           line.travelled_counts < curve1_end) {
                    corner_target = (int32_t)((int64_t)corner_base *
                        (curve1_end - line.travelled_counts) /
                        (curve1_end - curve1_release_start));
                }
            } else if (g_t5_curve_active) {
                corner_target = util_clamp_i64_to_i32(
                    (int64_t)corner_base *
                    TASK5_FINAL_CORNER_FF_PERCENT / 100);
            }
            corner_target = clamp_i32(corner_target,
                                      -TASK5_CORNER_FF_LIMIT,
                                      TASK5_CORNER_FF_LIMIT);
            int32_t current_abs = util_abs_i32(
                g_t5_corner_ff_filtered);
            int32_t target_abs = util_abs_i32(corner_target);
            int32_t corner_step =
                corner_target != 0 &&
                (g_t5_corner_ff_filtered == 0 ||
                 ((corner_target < 0) ==
                  (g_t5_corner_ff_filtered < 0))) &&
                target_abs > current_abs ?
                TASK5_CORNER_FF_ATTACK_PER_TICK :
                TASK5_CORNER_FF_RELEASE_PER_TICK;
            g_t5_corner_ff_filtered = task5_approach_value(
                g_t5_corner_ff_filtered, corner_target, corner_step);
        }
        int32_t motion_ff = g_t5_corner_ff_filtered;
        if (util_abs_i32(end_ff) > util_abs_i32(motion_ff)) {
            motion_ff = end_ff;
        }
        drive_ff = util_clamp_i64_to_i32((int64_t)drive_ff + motion_ff);
        g_ball_control_external_tilt_ff = clamp_i32(
            drive_ff, -TASK5_DRIVE_FF_LIMIT, TASK5_DRIVE_FF_LIMIT);
        if (ball_control_is_active()) {
            ball_control_step(dt_ms);
        }
        speed_service_step_10ms();

        ball_control_status_t ball = {0};
        bool ball_ok = ball_control_get_status(&ball) && ball.ball_valid;
        task5_record_ball_sample(ball_ok, &ball, dt_ms);

        if (line_ok) {
            g_t5_mask = line.digital_mask;
            g_t5_strong = line.strong_channel_count;
            g_t5_checkpoint_count = line.checkpoint_count;
            g_t5_checkpoint_detected = line.checkpoint_detected;
        }

        if (!g_t5_checkpoint_armed) {
            if (line_ok && line.line_detected &&
                !line.checkpoint_detected) {
                g_t5_checkpoint_clear_ms += dt_ms;
                if (g_t5_checkpoint_clear_ms >=
                    TASK5_CHECKPOINT_CLEAR_MS) {
                    g_t5_checkpoint_arm_count =
                        line.checkpoint_count;
                    g_t5_checkpoint_armed = true;
                }
            } else {
                g_t5_checkpoint_clear_ms = 0;
            }
            g_t5_checkpoint_detect_ms = 0;
        } else if (line_ok && line.checkpoint_detected) {
            g_t5_checkpoint_detect_ms += dt_ms;
        } else {
            g_t5_checkpoint_detect_ms = 0;
        }

        if (!line_ok || line.fault != LINE_TRACK_FAULT_NONE || !speed_ok) {
            task5_enter_fault(TASK5_FAULT_LINE);
        } else if (g_t5_ball_invalid_consecutive_ms >=
                   TASK5_BALL_LOST_STOP_MS) {
            task5_enter_fault(TASK5_FAULT_BALL_LOST);
        } else if (g_t5_checkpoint_armed &&
                   (line.checkpoint_count > g_t5_checkpoint_arm_count ||
                    g_t5_checkpoint_detect_ms >=
                        TASK5_CHECKPOINT_CONFIRM_MS)) {
            if (line.travelled_counts >=
                    g_t5_run_params.lap_learn_min_counts &&
                line.travelled_counts <=
                    g_t5_run_params.lap_learn_max_counts) {
                /* 同时更新本次锁存值和当前任务自己的RAM参数。 */
                g_t5_active_lap_estimate_counts = line.travelled_counts;
                g_t5_active_params->lap_estimate_counts =
                    line.travelled_counts;
            }
            task5_begin_braking(&speed);
        } else if (g_t5_elapsed_ms > g_t5_active_timeout_ms) {
            task5_enter_fault(TASK5_FAULT_TIMEOUT);
        } else if (!line.active) {
            task5_enter_fault(TASK5_FAULT_LINE);
        }
        return true;
    }

    if (g_t5_state == TASK5_BRAKING) {
        line_tracking_service_step_10ms();
        g_t5_brake_elapsed_ms += dt_ms;
        g_ball_control_external_tilt_ff = clamp_i32(
            g_t5_active_brake_ff, -g_ball_max_tilt_neg, g_ball_max_tilt);
        if (ball_control_is_active()) {
            ball_control_step(dt_ms);
        }

        int32_t brake_rpm = 0;
        if (g_t5_brake_elapsed_ms < TASK5_BRAKE_RAMP_MS) {
            brake_rpm = (int32_t)((int64_t)g_t5_brake_start_rpm *
                (TASK5_BRAKE_RAMP_MS - g_t5_brake_elapsed_ms) /
                TASK5_BRAKE_RAMP_MS);
        }
        if (!speed_service_set_target(brake_rpm, brake_rpm)) {
            task5_enter_fault(TASK5_FAULT_LINE);
            return true;
        }
        speed_service_step_10ms();

        ball_control_status_t ball = {0};
        speed_service_state_t speed = {0};
        bool ball_ok = ball_control_get_status(&ball) && ball.ball_valid;
        bool speed_ok = speed_service_get_state(&speed);
        task5_record_ball_sample(ball_ok, &ball, dt_ms);

        if (g_t5_ball_invalid_consecutive_ms >=
            TASK5_BALL_LOST_STOP_MS) {
            task5_enter_fault(TASK5_FAULT_BALL_LOST);
        } else if (speed_ok && util_abs_i32(speed.left_rpm) <=
                                   g_t5_run_params.wheel_stop_rpm &&
                   util_abs_i32(speed.right_rpm) <=
                       g_t5_run_params.wheel_stop_rpm) {
            g_t5_brake_stable_ms += dt_ms;
            if (g_t5_brake_stable_ms >= TASK5_BRAKE_STABLE_MS &&
                !line_tracking_service_is_active_snapshot()) {
                g_ball_control_external_tilt_ff = 0;
                g_t5_hold_elapsed_ms = 0;
                g_t5_hold_stable_ms = 0;
                g_t5_state = TASK5_HOLDING;
            }
        } else {
            g_t5_brake_stable_ms = 0;
        }

        if (g_t5_state == TASK5_BRAKING &&
            g_t5_brake_elapsed_ms > TASK5_BRAKE_TIMEOUT_MS) {
            task5_enter_fault(TASK5_FAULT_LINE);
        }
        return true;
    }

    if (g_t5_state == TASK5_HOLDING) {
        line_tracking_service_step_10ms();
        g_ball_control_external_tilt_ff = 0;
        if (ball_control_is_active()) {
            ball_control_step(dt_ms);
        }
        (void)speed_service_set_target(0, 0);
        speed_service_step_10ms();

        ball_control_status_t ball = {0};
        bool ball_ok = ball_control_get_status(&ball) && ball.ball_valid;
        task5_record_ball_sample(ball_ok, &ball, dt_ms);
        g_t5_hold_elapsed_ms += dt_ms;
        if (ball_ok && util_abs_i32(ball.error) <=
                       g_ball_params->tolerance_px) {
            g_t5_hold_stable_ms += dt_ms;
        } else {
            g_t5_hold_stable_ms = 0;
        }

        if (g_t5_hold_stable_ms >= TASK5_HOLD_STABLE_MS) {
            g_t5_state = TASK5_COMPLETE;
        } else if (g_t5_ball_invalid_consecutive_ms >=
                       TASK5_BALL_LOST_STOP_MS ||
                   g_t5_hold_elapsed_ms > TASK5_HOLD_TIMEOUT_MS) {
            task5_enter_fault(TASK5_FAULT_BALL_LOST);
        }
        return true;
    }

    return true;
}

bool task5_is_active(void)
{
    return g_t5_service_abort_pending ||
           g_t5_state == TASK5_PREPARE ||
           g_t5_state == TASK5_TRACKING ||
           g_t5_state == TASK5_BRAKING ||
           g_t5_state == TASK5_HOLDING;
}

bool task5_get_status(task5_status_t *out)
{
    if (out == NULL) {
        return false;
    }

    line_tracking_service_state_t line = {0};
    speed_service_state_t speed = {0};
    ball_control_status_t ball = {0};
    bool line_ok = line_tracking_service_get_state(&line);
    bool speed_ok = speed_service_get_state(&speed);
    bool ball_ok = ball_control_get_status(&ball);

    out->state = g_t5_state;
    out->fault = g_t5_fault;
    out->mode = g_t5_active_mode;
    out->elapsed_ms = g_t5_elapsed_ms;
    out->ball_target_x = g_t5_active_target_x;
    out->ball_x = ball_ok ? ball.ball_x : -1;
    out->ball_error = ball_ok ? ball.error : 0;
    out->ball_velocity = ball_ok ? ball.ball_vel : 0;
    out->ball_motor_output = ball_ok ? ball.motor_target : 0;
    out->max_abs_ball_error = g_t5_max_abs_ball_error;
    out->ball_accel_ff = g_ball_control_external_tilt_ff;
    out->travelled_counts = g_t5_travelled_counts;
    out->lap_estimate_counts = g_t5_active_lap_estimate_counts;
    out->approach_rpm = g_t5_approach_rpm;
    out->line_position = line_ok ? line.line_position : 0;
    out->turn_rpm = line_ok ? line.turn_rpm : 0;
    out->left_rpm = speed_ok ? speed.left_rpm : 0;
    out->right_rpm = speed_ok ? speed.right_rpm : 0;
    out->ball_violation_ms = g_t5_ball_violation_ms;
    out->ball_invalid_ms = g_t5_ball_invalid_ms;
    out->digital_mask = g_t5_mask;
    out->strong_channel = g_t5_strong;
    out->checkpoint_count = g_t5_checkpoint_count;
    out->line_fault = line_ok ? (uint8_t)line.fault :
                      (uint8_t)LINE_TRACK_FAULT_SPEED_STATE;
    out->checkpoint_detected = g_t5_checkpoint_detected;
    out->checkpoint_armed = g_t5_checkpoint_armed;
    out->ball_valid = ball_ok && ball.ball_valid;
    out->reached = g_t5_state == TASK5_COMPLETE;
    return true;
}

const char *task5_state_name(task5_state_t state)
{
    switch (state) {
    case TASK5_IDLE:     return "IDLE";
    case TASK5_PREPARE:  return "READY";
    case TASK5_TRACKING: return "TRACK";
    case TASK5_BRAKING:  return "BRAKE";
    case TASK5_HOLDING:  return "HOLD";
    case TASK5_COMPLETE: return "DONE";
    case TASK5_FAULT:    return "FAULT";
    default:             return "???";
    }
}
