/**
 * @file    tb6612.c
 * @brief   TB6612 双路直流电机驱动模块 (MSPM0G3507)
 * @note
 *   ── TB6612 芯片特性 ──
 *   - 双路 H 桥 (可独立驱动 2 个直流电机)
 *   - 供电电压: VM = 2.5~13.5V (电机), VCC = 2.7~5.5V (逻辑)
 *   - 最大连续电流: 1.2A / 通道
 *   - 控制模式: IN1/IN2 方向 + PWM 调速
 *
 *   ── 控制逻辑 (以通道 A 为例) ──
 *   | AIN1 | AIN2 | PWMA | 电机状态      |
 *   |------|------|------|--------------|
 *   |  0   |  0   |  X   | 滑行停止 (Hi-Z) |
 *   |  1   |  0   | PWM  | 正转 (CW)     |
 *   |  0   |  1   | PWM  | 反转 (CCW)    |
 *   |  1   |  1   |  X   | 短接制动       |
 *
 *   ── 硬件接线 ──
 *   PWMA  (左电机) → PA12 (TIMG0_CCP0, PWM 通道 0)
 *   AIN2             → PB19
 *   AIN1             → PB17
 *   STBY             → 已接 +5V (代码不控制, 始终使能)
 *   BIN1             → PA16
 *   BIN2             → PB24
 *   PWMB  (右电机) → PA13 (TIMG0_CCP1, PWM 通道 1)
 *
 *   ── PWM 配置 ──
 *   定时器: TIMG0 (SysConfig 名 "PWM_TB6612")
 *   时钟: BUSCLK = 40MHz
 *   周期: 2000 counts → PWM 频率 = 40MHz / 2000 = 20kHz (超出人耳听觉范围, 无声)
 *   极性: 低电平有效 (SysConfig 中 CC 匹配时输出低 → 反逻辑)
 *         cc_val = 0 → 占空比 100% (电机全速)
 *         cc_val = PERIOD → 占空比 0%   (电机停止)
 *   本模块使用反逻辑: speed=100 → cc_val=0, speed=0 → cc_val=PERIOD
 *
 *   ── 速度映射 ──
 *   输入: speed = -100 ~ +100 (抽象百分比)
 *   映射: cc_val = PERIOD × (100 - |speed|) / 100
 *   speed=-100 → cc=0  → 100% 反转
 *   speed=0    → cc=2000→ 0%   停止
 *   speed=+100 → cc=0  → 100% 正转
 */

#include "tb6612.h"
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  配置常量
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief 速度绝对值上限 (% 占空比) */
#define SPEED_MAX      100

/** @brief PWM 周期计数值 (40MHz / 2000 = 20kHz) */
#define PWM_PERIOD     2000

/* ═══════════════════════════════════════════════════════════════════════════
 *  引脚宏 (由 SysConfig GPIO_TB6612 映射)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── 方向脚 (通道 A = 物理左电机) ── */
#define AIN1_PORT      GPIO_TB6612_PIN_AIN1_PORT
#define AIN1_PIN       GPIO_TB6612_PIN_AIN1_PIN
#define AIN2_PORT      GPIO_TB6612_PIN_AIN2_PORT
#define AIN2_PIN       GPIO_TB6612_PIN_AIN2_PIN

/* ── 方向脚 (通道 B = 物理右电机) ── */
#define BIN1_PORT      GPIO_TB6612_PIN_BIN1_PORT
#define BIN1_PIN       GPIO_TB6612_PIN_BIN1_PIN
#define BIN2_PORT      GPIO_TB6612_PIN_BIN2_PORT
#define BIN2_PIN       GPIO_TB6612_PIN_BIN2_PIN

/* ── PWM 通道索引 (TIMG0) ── */
#define PWM_C0_IDX     GPIO_PWM_TB6612_C0_IDX   /* CC0 = PA12, 左电机 */
#define PWM_C1_IDX     GPIO_PWM_TB6612_C1_IDX   /* CC1 = PA13, 右电机 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  内部辅助: 方向设置
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  设置通道 A (左电机) 方向
 * @param  ain1  AIN1 电平 (0/1)
 * @param  ain2  AIN2 电平 (0/1)
 * @note   inline 优化: 频繁调用, 无函数调用开销
 */
static inline void set_dir_a(uint32_t ain1, uint32_t ain2)
{
    if (ain1) DL_GPIO_setPins(AIN1_PORT, AIN1_PIN);
    else      DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN);
    if (ain2) DL_GPIO_setPins(AIN2_PORT, AIN2_PIN);
    else      DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN);
}

/**
 * @brief  设置通道 B (右电机) 方向
 */
static inline void set_dir_b(uint32_t bin1, uint32_t bin2)
{
    if (bin1) DL_GPIO_setPins(BIN1_PORT, BIN1_PIN);
    else      DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN);
    if (bin2) DL_GPIO_setPins(BIN2_PORT, BIN2_PIN);
    else      DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  核心: 单电机控制
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  设置单个电机速度
 * @param  motor  通道标识: 'A' = 左电机, 其他 = 右电机
 * @param  speed  速度值 -100 ~ +100 (正=前进, 负=后退, 0=滑行停止)
 *
 * @note   控制流程:
 *         1) 钳位: 限制 speed 在 [-100, +100]
 *         2) 设置方向脚: 根据 speed 正负设置 IN1/IN2
 *         3) 计算 PWM 比较值: cc_val = PERIOD × (100 - |speed|) / 100
 *            - |speed|=100 → cc=0    → 100% 占空比 (全速)
 *            - |speed|=50  → cc=1000 → 50%  占空比
 *            - |speed|=0   → cc=2000 → 0%   占空比 (停止)
 *
 *         PWM 极性 (低电平有效):
 *           输出波形 = 低电平时长 / 周期
 *           cc_val 越小 → 低电平越长 → 占空比越大 → 电机越快
 */
static void set_motor(const char motor, int16_t speed)
{
    /* ── 钳位 ── */
    if (speed > SPEED_MAX)  speed = SPEED_MAX;
    if (speed < -SPEED_MAX) speed = -SPEED_MAX;

    /* 绝对值速度 */
    uint32_t abs_speed = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);

    /* PWM 比较值 (低电平有效, 反逻辑):
     * cc_val = PERIOD × (100 - abs_speed) / 100
     * 例: abs_speed=100 → cc_val=0    (100% 占空比)
     *     abs_speed=50  → cc_val=1000  (50%  占空比)
     *     abs_speed=0   → cc_val=2000  (0%   占空比) */
    uint32_t cc_val = PWM_PERIOD * (SPEED_MAX - abs_speed) / SPEED_MAX;

    if (motor == 'A') {
        /* ── 通道 A (左电机) ── */
        if (speed > 0) {
#if TB6612_REVERSE_DIRECTION
            set_dir_a(0, 1);   /* 正速度后退 */
#else
            set_dir_a(1, 0);   /* 正速度前进 */
#endif
        } else if (speed < 0) {
#if TB6612_REVERSE_DIRECTION
            set_dir_a(1, 0);   /* 负速度前进 */
#else
            set_dir_a(0, 1);   /* 负速度后退 */
#endif
        } else {
            set_dir_a(0, 0);   /* AIN1=0, AIN2=0 → 滑行停止 (Hi-Z) */
        }
        DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, cc_val, PWM_C0_IDX);
    } else {
        /* ── 通道 B (右电机) ── */
        if (speed > 0) {
#if TB6612_REVERSE_DIRECTION
            set_dir_b(0, 1);   /* 正速度后退 */
#else
            set_dir_b(1, 0);   /* 正速度前进 */
#endif
        } else if (speed < 0) {
#if TB6612_REVERSE_DIRECTION
            set_dir_b(1, 0);   /* 负速度前进 */
#else
            set_dir_b(0, 1);   /* 负速度后退 */
#endif
        } else {
            set_dir_b(0, 0);   /* BIN1=0, BIN2=0 → 滑行停止 (Hi-Z) */
        }
        DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, cc_val, PWM_C1_IDX);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  公开接口
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  初始化 TB6612 驱动
 * @note   方向脚已由 SysConfig 初始化为输出低电平
 *         SYSCFG_DL_PWM_TB6612_init() 已配置 PWM 模块
 *         本函数设置方向为停止, 启动 PWM 计数器
 *         应在 SYSCFG_DL_init() 之后调用
 */
void tb6612_init(void)
{
    /* ── 初始化为停止状态 (滑行) ── */
    set_dir_a(0, 0);  /* AIN1=0, AIN2=0 */
    set_dir_b(0, 0);  /* BIN1=0, BIN2=0 */

    /* PWM 占空比 0%: cc_val = PERIOD 已由 SysConfig 初始化设置 */

    /* ── 启动 PWM 计数器 ──
     * 启动后输出波形: 占空比 0%, 电机停止
     * 后续 set_motor() 可动态修改比较值 */
    DL_TimerG_startCounter(PWM_TB6612_INST);
}

/**
 * @brief  设置左电机速度
 * @param  speed  -100 ~ +100 (正=前进, 负=后退)
 */
void tb6612_set_left_speed(int16_t speed)
{
    set_motor('A', speed);
}

/**
 * @brief  设置右电机速度
 * @param  speed  -100 ~ +100 (正=前进, 负=后退)
 */
void tb6612_set_right_speed(int16_t speed)
{
    set_motor('B', speed);
}

/**
 * @brief  同时设置左右电机速度
 * @param  left   左电机速度 -100 ~ +100
 * @param  right  右电机速度 -100 ~ +100
 * @note   这是速度闭环控制中最常用的接口
 */
void tb6612_set_speed(int16_t left, int16_t right)
{
    set_motor('A', left);
    set_motor('B', right);
}

/**
 * @brief  两电机滑行停止 (Hi-Z 高阻态)
 * @note   IN1=IN2=0, 电机关闭驱动, 自由惯性滑行
 *         PWM 比较值设置为 PERIOD (0% 占空比)
 *         与 tb6612_brake() 的区别: 无制动力, 靠摩擦力自然减速
 */
void tb6612_stop(void)
{
    set_dir_a(0, 0);  /* 通道 A: 滑行 */
    set_dir_b(0, 0);  /* 通道 B: 滑行 */

    /* 占空比清零 (停止向电机供电) */
    DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, PWM_PERIOD, PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, PWM_PERIOD, PWM_C1_IDX);
}

/**
 * @brief  两电机制动 (短接制动)
 * @note   IN1=IN2=1, 电机绕组短接, 产生反向电动势制动
 *         制动效果: 电机快速减速, 类似"急刹车"
 *         与 tb6612_stop() 的区别: 有制动力, 快速停止
 *         ⚠ 长时间制动可能导致电机过热
 */
void tb6612_brake(void)
{
    /* 短接制动: AIN1=AIN2=1, BIN1=BIN2=1 */
    set_dir_a(1, 1);  /* 通道 A: 短接 */
    set_dir_b(1, 1);  /* 通道 B: 短接 */

    /* 占空比清零 (PWM 无效, 制动状态下不调制) */
    DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, PWM_PERIOD, PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_TB6612_INST, PWM_PERIOD, PWM_C1_IDX);
}
