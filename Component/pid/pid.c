/**
 * @file    pid.c
 * @brief   增量式 PID 控制器实现 (整数运算, 无浮点)
 * @note
 *   ── 增量式 PID 公式 ──
 *   Δu[k] = Kp * (e[k] - e[k-1]) + Ki * e[k] + Kd * (e[k] - 2*e[k-1] + e[k-2])
 *   u[k]  = clamp(u[k-1] + Δu[k], out_min, out_max)
 *
 *   其中: e[k] = target - measured (当前误差)
 *         e[k-1] = 上一次误差
 *         e[k-2] = 上上次误差
 *
 *   ── 增量式 vs 位置式 ──
 *   增量式优点:
 *     1. 天然抗积分饱和: 只需对输出限幅, 无需额外积分分离逻辑
 *     2. 无扰动切换: 手动→自动切换时输出不会突变
 *     3. 计算量小: 增量公式中只用到最近 3 次误差
 *
 *   ── 千倍整数表示法 ──
 *   为避免 MCU 浮点运算开销, PID 系数以千倍整数存储:
 *     kp_milli = Kp × 1000,  ki_milli = Ki × 1000,  kd_milli = Kd × 1000
 *   内部计算后除以 1000 恢复真实值
 *   四舍五入处理: 正数 +500/÷1000, 负数 -500/÷1000
 *
 *   ── 输出限幅 ──
 *   在累积输出值 (output_milli) 上做硬限幅, 防止积分 windup
 */

#include "pid/pid.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  内部辅助函数
 * ═══════════════════════════════════════════════════════════════════════════ */
/**
 * @brief  32 位有符号整数限幅
 * @param  value      输入值
 * @param  min_value  下限
 * @param  max_value  上限
 * @return 限幅后的值
 */
static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

/**
 * @brief  千倍整数 → 实际整数 (四舍五入)
 * @param  value_milli  千倍值 (如 80456 → 80)
 * @return 四舍五入后的整数值
 * @note   正数 +500 再 ÷1000, 负数 -500 再 ÷1000
 *         例: 80456 → (80456+500)/1000 = 80
 *             79500 → (79500+500)/1000 = 80
 *             79499 → (79499+500)/1000 = 79
 */
static int32_t milli_to_i32_round(int32_t value_milli)
{
    if (value_milli >= 0) {
        return (value_milli + 500) / 1000;
    }
    return (value_milli - 500) / 1000;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  公开接口
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  初始化增量式 PID 控制器
 * @param  pid        控制器指针
 * @param  kp_milli   比例系数 × 1000 (例: 80 = Kp 0.080)
 * @param  ki_milli   积分系数 × 1000
 * @param  kd_milli   微分系数 × 1000
 * @param  out_min    输出下限 (例: -50)
 * @param  out_max    输出上限 (例: +50)
 * @note   调用后会立即复位所有内部状态 (输出清零, 历史误差清零)
 *         调用 pid_inc_set_gain() 可后续调整系数 (不复位状态)
 */
void pid_inc_init(pid_inc_t *pid, int32_t kp_milli, int32_t ki_milli,
                  int32_t kd_milli, int32_t out_min, int32_t out_max)
{
    if (pid == 0) {
        return;  /* 空指针保护 */
    }

    /* ── 存储系数 (千倍整数) ── */
    pid->kp_milli = kp_milli;
    pid->ki_milli = ki_milli;
    pid->kd_milli = kd_milli;

    /* ── 输出限幅范围 ── */
    pid->out_min  = out_min;
    pid->out_max  = out_max;

    /* ── 复位所有内部状态 ── */
    pid_inc_reset(pid);
}

/**
 * @brief  运行时调整 PID 系数 (不复位内部状态)
 * @param  pid        控制器指针
 * @param  kp_milli   新的比例系数 × 1000
 * @param  ki_milli   新的积分系数 × 1000
 * @param  kd_milli   新的微分系数 × 1000
 * @note   只修改系数, 不改变输出值和历史误差
 *         可用于在线参数整定 (如通过串口指令动态修改)
 */
void pid_inc_set_gain(pid_inc_t *pid, int32_t kp_milli, int32_t ki_milli,
                      int32_t kd_milli)
{
    if (pid == 0) {
        return;
    }

    pid->kp_milli = kp_milli;
    pid->ki_milli = ki_milli;
    pid->kd_milli = kd_milli;
}

void pid_inc_set_output_limit(pid_inc_t *pid, int32_t out_min, int32_t out_max)
{
    if (pid == 0) {
        return;
    }

    if (out_min > out_max) {
        int32_t tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->output_milli = clamp_i32(pid->output_milli,
                                  pid->out_min * 1000,
                                  pid->out_max * 1000);
    pid->output = milli_to_i32_round(pid->output_milli);
}

/**
 * @brief  复位 PID 控制器内部状态
 * @param  pid  控制器指针
 * @note   清零: 输出值、历史误差 e[k-1]、e[k-2]
 *         通常在目标变更/模式切换时调用, 避免历史误差影响新控制目标
 */
void pid_inc_reset(pid_inc_t *pid)
{
    if (pid == 0) {
        return;
    }

    /* 输出清零 → 电机停转 (PWM = 0, 保持当前位置) */
    pid->output       = 0;
    pid->output_milli = 0;  /* 千倍输出也清零 */

    /* 历史误差清零 → 下次计算仅依赖当前误差 */
    pid->err_1 = 0;  /* e[k-1] */
    pid->err_2 = 0;  /* e[k-2] */
}

/**
 * @brief  执行一次增量式 PID 计算
 * @param  pid       控制器指针
 * @param  target    目标值 (setpoint, 例: 目标 RPM)
 * @param  measured  测量值 (process variable, 例: 实测 RPM)
 * @return PID 输出值 (已限幅, 在 [out_min, out_max] 之间)
 *
 * @note   计算流程:
 *         1. 误差:         e  = target - measured
 *         2. 增量:         Δu = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2)
 *         3. 累加并限幅:   u  = clamp(u + Δu, out_min, out_max)
 *         4. 保存历史:     e2 = e1, e1 = e
 *
 *         首次调用时 e1=e2=0 → Δu = Kp*e + Ki*e + Kd*e
 *         = (Kp+Ki+Kd) * e   (退化为 P 控制)
 */
int32_t pid_inc_compute(pid_inc_t *pid, int32_t target, int32_t measured)
{
    if (pid == 0) {
        return 0;
    }

    /* ── 第 1 步: 计算当前误差 ── */
    int32_t err = target - measured;

    /* ── 第 2 步: 增量式 PID 计算 ── */
    int64_t delta_milli = 0;

    /* 比例项: Kp * (e[k] - e[k-1]) */
    delta_milli += (int64_t)pid->kp_milli * (err - pid->err_1);

    /* 积分项: Ki * e[k] */
    delta_milli += (int64_t)pid->ki_milli * err;

    /* 微分项: Kd * (e[k] - 2*e[k-1] + e[k-2]) */
    delta_milli += (int64_t)pid->kd_milli * (err - 2 * pid->err_1 + pid->err_2);

    /* ── 第 3 步: 累加并限幅 ──
     * 只对最终输出做 clamp，不把超限增量直接清零。
     * 这样输出可以正常到达 out_min/out_max，避免接近限幅时响应被提前削弱。
     */
    int32_t min_milli = pid->out_min * 1000;
    int32_t max_milli = pid->out_max * 1000;
    int64_t next_milli = (int64_t)pid->output_milli + delta_milli;

    if (next_milli > max_milli) {
        pid->output_milli = max_milli;
    } else if (next_milli < min_milli) {
        pid->output_milli = min_milli;
    } else {
        pid->output_milli = (int32_t)next_milli;
    }

    /* 千倍 → 实际值。电机死区/前馈补偿放到电机输出层处理，PID 保持纯净。 */
    pid->output = milli_to_i32_round(pid->output_milli);

    /* ── 第 4 步: 保存历史误差 ── */
    pid->err_2 = pid->err_1;  /* e[k-2] ← e[k-1] */
    pid->err_1 = err;         /* e[k-1] ← e[k]   */

    return pid->output;
}

/**
 * @brief  获取 PID 控制器当前输出值 (不执行计算)
 * @param  pid  控制器指针
 * @return 当前输出值, pid==NULL 时返回 0
 */
int32_t pid_inc_get_output(const pid_inc_t *pid)
{
    return (pid == 0) ? 0 : pid->output;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  位置式 PID：用于 yaw/循线等上层位置量闭环
 * ═══════════════════════════════════════════════════════════════════════════ */

void pid_pos_init(pid_pos_t *pid, int32_t kp_milli, int32_t ki_milli,
                  int32_t kd_milli, int32_t out_min, int32_t out_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp_milli = kp_milli;
    pid->ki_milli = ki_milli;
    pid->kd_milli = kd_milli;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_pos_reset(pid);
}

void pid_pos_set_gain(pid_pos_t *pid, int32_t kp_milli, int32_t ki_milli,
                      int32_t kd_milli)
{
    if (pid == 0) {
        return;
    }

    pid->kp_milli = kp_milli;
    pid->ki_milli = ki_milli;
    pid->kd_milli = kd_milli;
}

void pid_pos_set_output_limit(pid_pos_t *pid, int32_t out_min, int32_t out_max)
{
    if (pid == 0) {
        return;
    }

    if (out_min > out_max) {
        int32_t tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_milli = clamp_i32(pid->integral_milli,
                                    pid->out_min * 1000,
                                    pid->out_max * 1000);
    pid->output = clamp_i32(pid->output, pid->out_min, pid->out_max);
}

void pid_pos_reset(pid_pos_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->output = 0;
    pid->integral_milli = 0;
    pid->last_err = 0;
    pid->first_run = 1U;
}

int32_t pid_pos_compute_error(pid_pos_t *pid, int32_t err)
{
    if (pid == 0) {
        return 0;
    }

    /* 内部积分：ki * err 累加到 integral_milli，再做 anti-windup 限幅。 */
    int64_t next_integral = (int64_t)pid->integral_milli +
                            (int64_t)pid->ki_milli * err;
    int32_t min_milli = pid->out_min * 1000;
    int32_t max_milli = pid->out_max * 1000;

    if (next_integral > max_milli) {
        pid->integral_milli = max_milli;
    } else if (next_integral < min_milli) {
        pid->integral_milli = min_milli;
    } else {
        pid->integral_milli = (int32_t)next_integral;
    }

    return pid_pos_compute_with_integral(pid, err, pid->integral_milli, 0);
}

int32_t pid_pos_compute_with_integral(pid_pos_t *pid,
                                       int32_t err,
                                       int32_t integral_milli,
                                       int32_t *derr_out)
{
    if (pid == 0) {
        return 0;
    }

    /* 误差变化（首周期为 0，避免 D 项突变）。 */
    int32_t derr = 0;
    if (pid->first_run) {
        pid->first_run = 0U;
    } else {
        derr = err - pid->last_err;
    }
    if (derr_out != 0) {
        *derr_out = derr;
    }

    int32_t min_milli = pid->out_min * 1000;
    int32_t max_milli = pid->out_max * 1000;

    int64_t output_milli = 0;
    output_milli += (int64_t)pid->kp_milli * err;
    output_milli += integral_milli;
    output_milli += (int64_t)pid->kd_milli * derr;

    if (output_milli > max_milli) {
        output_milli = max_milli;
    } else if (output_milli < min_milli) {
        output_milli = min_milli;
    }

    int32_t output = milli_to_i32_round((int32_t)output_milli);
    pid->output = clamp_i32(output, pid->out_min, pid->out_max);
    pid->last_err = err;

    return pid->output;
}

int32_t pid_pos_get_integral_milli(const pid_pos_t *pid)
{
    return (pid == 0) ? 0 : pid->integral_milli;
}

void pid_pos_set_integral_milli(pid_pos_t *pid, int32_t integral_milli)
{
    if (pid != 0) {
        pid->integral_milli = integral_milli;
    }
}

int32_t pid_pos_get_ki_milli(const pid_pos_t *pid)
{
    return (pid == 0) ? 0 : pid->ki_milli;
}

int32_t pid_pos_get_out_max(const pid_pos_t *pid)
{
    return (pid == 0) ? 0 : pid->out_max;
}

int32_t pid_pos_get_derr_start(pid_pos_t *pid, int32_t err)
{
    if (pid == 0) {
        return 0;
    }

    if (pid->first_run) {
        pid->first_run = 0U;
        return 0;
    }

    return err - pid->last_err;
}

int32_t pid_pos_get_output(const pid_pos_t *pid)
{
    return (pid == 0) ? 0 : pid->output;
}
