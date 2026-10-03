/**
 * @file    encoder.c
 * @brief   双路增量式编码器读取模块 (MSPM0G3507)
 * @note
 *   ── 编码器速度换算 ──
 *   本模块只提供原始累计值和增量值；每圈计数由 speed_service 从
 *   当前菜单任务的独立 .speed.counts_per_rev 参数读取。
 *
 *   ── 硬件映射 ──
 *   物理右轮: TIMG8 硬件 QEI (正交编码器接口)
 *     A 相 = PA26 (TIMG8_CCP0), B 相 = PA27 (TIMG8_CCP1)
 *     优点: 硬件自动 4 倍频, 无需 CPU 干预
 *     缺点: TIMG12 不支持 QEI, 另一路只能软件解码
 *
 *   物理左轮: GPIO 双边沿软件解码
 *     A 相 = PA25, B 相 = PA14
 *     原理: 在 GROUP1_IRQHandler 中检测 A/B 两相电平变化,
 *           通过 16 项查表实现正交解码 (4 倍频)
 *
 *   ── 数据输出映射 (encoder_get_data 内部交换) ──
 *   实测: QEI 通道连接物理右轮, GPIO 通道连接物理左轮
 *   为保持 API 语义一致性 (left_count = 左轮, right_count = 右轮),
 *   encoder_get_data() 内部进行了左右交换
 *
 *   ── 方向修正 ──
 *   右轮硬件 QEI: 向前转动 → 计数减少, 通过 RIGHT_ENCODER_DIR = -1 取反
 *   左轮软件解码: 查表结果直接为正向 (向前 → 计数增加)
 *
 *   ── QEI 16 位计数器溢出处理 ──
 *   TIMG8 计数器为 16 位 (0~65535), 使用半程法处理溢出:
 *     diff > +32767 → 实际为负溢出 (diff -= 65536)
 *     diff < -32768 → 实际为正溢出 (diff += 65536)
 *   半程法假设两次读取之间计数变化不超过 32767, 在 10ms 周期下完全满足
 */

#include "encoder/encoder.h"
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  左轮 (硬件 QEI) 相关常量
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief QEI 计数器最大值 (16 位) */
#define LEFT_QEI_MAX_COUNT      (65535U)

/** @brief 半程阈值: 用于 16 位计数器溢出检测 */
#define LEFT_QEI_HALF_RANGE     (32768)

/* ═══════════════════════════════════════════════════════════════════════════
 *  右轮 (GPIO 软件解码) 相关常量
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief 右轮编码器 A 相引脚 (PA14) */
#define RIGHT_A_PIN             GPIO_ENCODER_RIGHT_PIN_RIGHT_A_PIN

/** @brief 右轮编码器 B 相引脚 (PA25) */
#define RIGHT_B_PIN             GPIO_ENCODER_RIGHT_PIN_RIGHT_B_PIN

/** @brief A/B 相的位掩码 OR */
#define RIGHT_PINS              (RIGHT_A_PIN | RIGHT_B_PIN)

/**
 * @brief 右轮 QEI 方向修正系数
 * @note  实测: 右轮向前转动 → 硬件 QEI 计数减少 → 取反(-1)使其增加
 */
#define RIGHT_ENCODER_DIR       (-1)

/* ═══════════════════════════════════════════════════════════════════════════
 *  全局状态变量
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── 左轮 QEI ── */
static int32_t  g_left_count       = 0;    /* 累计计数 (含溢出补偿) */
static uint16_t g_left_last_raw    = 0;    /* 上次 QEI 原始值 (16位) */
static int32_t  g_left_last_report = 0;    /* 上次报告给调用者的累计值 */

/* ── 右轮 GPIO ──
 * volatile: ISR 中修改, 任务上下文读取, 保证可见性 */
static volatile int32_t  g_right_count       = 0;    /* 累计计数 */
static volatile uint8_t  g_right_last_state  = 0;    /* 上次 A/B 电平状态 (bit1=A, bit0=B) */
static int32_t           g_right_last_report = 0;    /* 上次报告给调用者的累计值 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  右轮 GPIO 正交解码
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  读取右轮编码器 AB 相当前电平
 * @return 状态编码: bit1=A, bit0=B (范围 0~3)
 *         例: A=高 B=低 → 返回 2 (二进制 10)
 */
static uint8_t read_right_state(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_ENCODER_RIGHT_PORT, RIGHT_PINS);
    uint8_t a = (pins & RIGHT_A_PIN) ? 1U : 0U;
    uint8_t b = (pins & RIGHT_B_PIN) ? 1U : 0U;

    /* 组合: bit1 = A, bit0 = B → 00/01/10/11 */
    return (uint8_t)((a << 1) | b);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  左轮 QEI 计数器更新
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  读取 QEI 硬件计数器并更新累计值 (处理 16 位溢出)
 * @note   关键算法: 半程法溢出检测
 *         2 的补码 16 位范围: 0 ~ 65535
 *         若 diff > 32767 (半程), 实际为负溢出:
 *           now = 100, last = 65400 → diff = 100-65400 = -65300 (int32_t)
 *           -65300 < -32768 → diff += 65536 = 236 (正向经过了 65536→0 的回绕)
 *         若 diff < -32768, 实际为正溢出
 */
static void update_left_count(void)
{
    /* 读取当前 QEI 计数器 (16 位) */
    uint16_t now  = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_LEFT_INST);
    int32_t  diff = (int32_t)now - (int32_t)g_left_last_raw;

    /* ── 半程法溢出检测 ── */
    if (diff > LEFT_QEI_HALF_RANGE) {
        /* 正数过大 → 实际为负方向溢出 (now < last, 绕过了 0) */
        diff -= (int32_t)(LEFT_QEI_MAX_COUNT + 1U);  /* -65536 */
    } else if (diff < -LEFT_QEI_HALF_RANGE) {
        /* 负数过大 → 实际为正方向溢出 (now > last, 绕过了 65535) */
        diff += (int32_t)(LEFT_QEI_MAX_COUNT + 1U);  /* +65536 */
    }

    /* 累加增量 */
    g_left_count  += diff;
    g_left_last_raw = now;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  公开接口
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  编码器初始化
 * @note   初始化所有状态变量, 使能右轮 GPIO 双边沿中断
 *         启动左轮 QEI 计数器
 *         左轮 GPIO 中断已由 SysConfig 使能
 */
void encoder_init(void)
{
    /* ── 左轮 QEI 初始化 ── */
    g_left_last_raw    = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_LEFT_INST);
    g_left_count       = 0;
    g_left_last_report = 0;

    /* ── 右轮 GPIO 初始化 ── */
    g_right_last_state  = read_right_state();  /* 记录当前电平状态 */
    g_right_count       = 0;
    g_right_last_report = 0;

    /* ── 使能右轮 GPIO 双边沿中断 ──
     * 中断优先级 3: 低于速度闭环定时器 (保证控制周期)
     * 中断触发: A 相或 B 相任一引脚电平变化 (上升沿 + 下降沿) */
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_PORT, RIGHT_PINS);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);
    NVIC_SetPriority(GPIO_ENCODER_RIGHT_INT_IRQN, 3);
    NVIC_EnableIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);

    /* ── 启动左轮 QEI 计数器 ── */
    DL_TimerG_startCounter(QEI_ENCODER_LEFT_INST);
}

/**
 * @brief  零点复位编码器累计计数值
 * @note   通常在速度闭环控制复位时调用 (如目标速度变为 0 时)
 *         先停止计数器避免在清零过程中产生新计数
 */
void encoder_reset(void)
{
    /* ── 左轮 QEI 复位 ── */
    DL_TimerG_stopCounter(QEI_ENCODER_LEFT_INST);
    DL_TimerG_setTimerCount(QEI_ENCODER_LEFT_INST, 0);
    g_left_last_raw    = 0;
    g_left_count       = 0;
    g_left_last_report = 0;

    /* ── 右轮 GPIO 复位 ── */
    g_right_last_state  = read_right_state();
    g_right_count       = 0;
    g_right_last_report = 0;

    /* ── 重新启动计数器 ── */
    DL_TimerG_startCounter(QEI_ENCODER_LEFT_INST);
}

/**
 * @brief  获取编码器累计值和增量
 * @param  data  输出参数, 编码器数据快照
 * @note
 *   ── 左右交换映射 ──
 *   硬件连接: QEI 通道 (PA26/PA27) → 物理右轮
 *             GPIO 通道 (PA25/PA14) → 物理左轮
 *   但 API 期望 left_count/left_delta = 左轮数据
 *   因此这里做了交换: 让调用者无感知硬件差异
 *
 *   ── delta 计算 ──
 *   delta = 当前累计值 - 上次报告值
 *   delta 可正可负, 正值 = 正转增量, 负值 = 反转增量
 */
void encoder_get_data(encoder_data_t *data)
{
    if (data == 0) {
        return;  /* 空指针保护 */
    }

    /* 更新左轮 QEI 累计值 */
    update_left_count();

    /* ── 读取当前累计值并计算增量 ──
     * g_left_count      : QEI 通道 → 物理右轮
     * g_right_count     : GPIO 通道 → 物理左轮
     * 交换映射使得:
     *   data->left_*   = GPIO 通道 (物理左轮)
     *   data->right_*  = QEI 通道 (物理右轮) */
    int32_t qei_now   = g_left_count;    /* QEI 通道当前值 */
    int32_t gpio_now  = g_right_count;   /* GPIO 通道当前值 */

    /* 输出: 左=GPIO(物理左), 右=QEI(物理右) */
    data->left_count  = gpio_now;
    data->right_count = qei_now;

    /* 增量 = 当前值 - 上次报告值 */
    data->left_delta  = gpio_now - g_right_last_report;
    data->right_delta = qei_now  - g_left_last_report;

    /* 更新上次报告值 */
    g_left_last_report  = qei_now;
    g_right_last_report = gpio_now;
}

/**
 * @brief  检查右轮编码器 GPIO 中断是否待处理
 * @return 非 0 = 有待处理中断, 0 = 无
 * @note   用于 GROUP1_IRQHandler 中判断中断源
 */
int encoder_right_int_is_pending(void)
{
    return (DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_RIGHT_PORT,
            RIGHT_PINS) != 0U);
}

/**
 * @brief  右轮编码器 GPIO 正交解码中断处理
 * @note   必须在 GROUP1_IRQHandler 中调用
 *
 *   ── 正交解码原理 (4 倍频查表法) ──
 *   编码器 A/B 两相输出 90° 相位差方波, 每个完整周期有 4 个边沿:
 *
 *   正向: AB序列 = 00→10→11→01→00  (顺时针, 查表 +1)
 *   反向: AB序列 = 00→01→11→10→00  (逆时针, 查表 -1)
 *
 *   查表索引: (上次状态 << 2) | 当前状态 = 4 位索引 (0~15)
 *   查表结果: +1=正转一步, -1=反转一步, 0=无效跳变 (丢步或噪声)
 *
 *   状态编码 (bit1=A, bit0=B):
 *     ┌──────────────────────────────────────┐
 *     │ 状态值 │ A  │ B  │ 含义              │
 *     │   0    │ 0  │ 0  │ A低 B低           │
 *     │   1    │ 0  │ 1  │ A低 B高           │
 *     │   2    │ 1  │ 0  │ A高 B低           │
 *     │   3    │ 1  │ 1  │ A高 B高           │
 *     └──────────────────────────────────────┘
 *
 *   quadrature_table[16] 含义: 行=上次状态, 列=当前状态
 *   quadrature_table[old<<2|new]:
 *      0=无效跳变/未变化, +1=正向步进, -1=反向步进
 */
void encoder_right_irq_handler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_RIGHT_PORT, RIGHT_PINS);

    /* 确认中断来自编码器引脚 */
    if ((pending & RIGHT_PINS) != 0U) {
        uint8_t state = read_right_state();  /* 当前 A/B 电平状态 (0~3) */

        /* 查表索引: (上次状态 << 2) | 当前状态 → 0~15 */
        uint8_t index = (uint8_t)((g_right_last_state << 2) | state);

        /* ── 正交解码查表 ──
         * 解释:
         *   索引 0b0000 (old=0,new=0): 无变化 → 0
         *   索引 0b0010 (old=0,new=2): 00→10, A升, 正向 → +1
         *   索引 0b0001 (old=0,new=1): 00→01, B升, 反向 → -1
         *   索引 0b1011 (old=2,new=3): 10→11, B升, 正向 → +1
         *   ... 等共 16 种组合 */
        static const int8_t quad_table[16] = {
             0, -1,  1,  0,   /* old=0: 00→{00,01,10,11} */
             1,  0,  0, -1,   /* old=1: 01→{00,01,10,11} */
            -1,  0,  0,  1,   /* old=2: 10→{00,01,10,11} */
             0,  1, -1,  0    /* old=3: 11→{00,01,10,11} */
        };

        /* 累加增量: × RIGHT_ENCODER_DIR 修正方向 */
        g_right_count += (int32_t)RIGHT_ENCODER_DIR * quad_table[index];

        /* 更新上一次状态 */
        g_right_last_state = state;

        /* 清除已处理的中断标记 */
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_PORT,
                                     pending & RIGHT_PINS);
    }
}
