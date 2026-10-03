/**
 * @file gray_sensor.c
 * @brief 八路灰度传感器驱动实现：地址切换、ADC 采样、归一化与滞回二值化。
 *
 * @details
 * 硬件结构：传感器为无 MCU 基础款，板载 74HC4051 八选一模拟多路复用器。
 *   - PB0/PB1/PB10 输出 3 位地址（000~111）依次选择 CH1~CH8
 *   - PB25 (ADC0_CH4) 采集选中通道的模拟输出
 *   - 白底反射强 → ADC 值大；黑线反射弱 → ADC 值小
 *
 * 采集流程（每路）：
 *   切换地址 → 等待稳定(10μs) → ADC 采样 8 次取平均 → 得到一路 raw 值
 * 8 路共约 6.4ms，在 10ms 控制周期内可完成。
 *
 * 数据处理（gray_sensor_process）：
 *   raw + 标定值(white/black) → 归一化(0~4095) → 滞回二值化(digital_mask)
 *   归一化公式：normalized = (raw - black) × 4095 / (white - black)
 *   消除各路 LED/光电管/安装差异，使所有通道在同一量纲下比较。
 *
 * 本文件不依赖 FreeRTOS，可被任意任务上下文调用。service 层在 10ms
 * 周期中调用 gray_sensor_read() 完成一次完整的"采样+处理"。
 */
#include "gray_sensor/gray_sensor.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>
#include <string.h>

/* ── GPIO 引脚定义：74HC4051 三位地址线 ────────────────────────────────── */
#define GRAY_AD0_PIN GPIO_GRAY_ADDRESS_PIN_AD0_PIN  /* 地址 bit0 → CH 奇偶 */
#define GRAY_AD1_PIN GPIO_GRAY_ADDRESS_PIN_AD1_PIN  /* 地址 bit1 → CH 高低半 */
#define GRAY_AD2_PIN GPIO_GRAY_ADDRESS_PIN_AD2_PIN  /* 地址 bit2 → CH 前后半 */
#define GRAY_ADDRESS_PINS (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)

/* ── 标定值全局常量：从 gray_sensor_config.h 宏初始化 ──────────────────────
 * 这两个数组是逐路标定的白底/黑线 ADC 基准值，用于归一化计算。
 * 每路传感器的 LED 发光强度、光电管灵敏度、安装高度不同，所以各路值不同。
 * 详见 gray_sensor_config.h 中的标定说明。 */
static const uint16_t g_default_white[GRAY_SENSOR_CHANNEL_COUNT] =
    GRAY_SENSOR_WHITE_VALUES;
static const uint16_t g_default_black[GRAY_SENSOR_CHANNEL_COUNT] =
    GRAY_SENSOR_BLACK_VALUES;

/* 滞回二值化的状态记忆：保存上一帧的 digital_mask（1=白/0=黑）。
 * 处于滞回区间（既不够白也不够黑）的通道保持上一帧状态，防止边界抖动。 */
static uint8_t g_digital_mask = 0xFFU;

/* ========================================================================== */
/*  硬件层：74HC4051 地址切换与 ADC 单次读取                                   */
/* ========================================================================== */

/**
 * @brief 切换 74HC4051 多路复用器到指定通道。
 * @param channel 通道号 0~7，对应 CH1~CH8
 *
 * @details 将 channel 的 3 个 bit 分别输出到 AD0/AD1/AD2 引脚：
 *   channel & 0x01 → AD0（bit0）
 *   channel & 0x02 → AD1（bit1）
 *   channel & 0x04 → AD2（bit2）
 * 先清所有地址线再按位设置，然后等待 GRAY_SENSOR_ADDRESS_SETTLE_US(10μs)
 * 让模拟输出稳定，避免采到切换瞬间的毛刺。
 */
static void gray_sensor_select_channel(uint8_t channel)
{
    uint32_t set_mask = 0U;

    if ((channel & 0x01U) != 0U) {   /* bit0 → AD0 */
        set_mask |= GRAY_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U) {   /* bit1 → AD1 */
        set_mask |= GRAY_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U) {   /* bit2 → AD2 */
        set_mask |= GRAY_AD2_PIN;
    }

    /* 先清所有地址线（选择 channel 0 状态），再设需要的位 */
    DL_GPIO_clearPins(GPIO_GRAY_ADDRESS_PORT, GRAY_ADDRESS_PINS);
    if (set_mask != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_ADDRESS_PORT, set_mask);
    }

    /* 等待 74HC4051 模拟输出稳定，避免采到切换瞬间的毛刺 */
    delay_cycles((CPUCLK_FREQ / 1000000U) *
                 GRAY_SENSOR_ADDRESS_SETTLE_US);
}

/**
 * @brief 用 SysTick 倒计数特性计算流逝的 CPU 周期数。
 * @param start 起始 SysTick->VAL
 * @param now   当前 SysTick->VAL
 * @return 从 start 到 now 经过的 CPU 周期数
 *
 * @details SysTick 是倒计数器（LOAD → 0 循环），所以 now < start 为正常。
 * 如果 now > start 说明期间发生了重装载（start 在上一轮末尾），
 * 需要加上一个完整周期长度来修正。用于 ADC 超时检测，不依赖 FreeRTOS。
 */
static uint32_t gray_sensor_elapsed_cycles(uint32_t start, uint32_t now)
{
    uint32_t period = SysTick->LOAD + 1U;
    return (start >= now) ? (start - now) : (start + period - now);
}

/**
 * @brief 执行一次 ADC 转换并读取结果，带超时保护。
 * @param value 输出：12 位 ADC 原始值
 * @return true=转换成功；false=超时（ADC 硬件异常）
 *
 * @details 启动转换后用 SysTick 轮询等待 MEM0 结果寄存器就绪标志。
 * 超时阈值 = GRAY_SENSOR_ADC_TIMEOUT_US(100μs)，超时则禁用转换并返回 false。
 * 调用方（scan_raw）在超时时会清零整帧 raw 并返回 false，service 据此报 ADC 故障。
 */
static bool gray_sensor_adc_read_once(uint16_t *value)
{
    if (value == NULL) {
        return false;
    }

    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    DL_ADC12_startConversion(ADC_GRAY_INST);

    /* 用 SysTick 倒计数做超时检测，不依赖 FreeRTOS tick */
    uint32_t start = SysTick->VAL;
    uint32_t timeout_cycles = (CPUCLK_FREQ / 1000000U) *
                              GRAY_SENSOR_ADC_TIMEOUT_US;
    while (DL_ADC12_getRawInterruptStatus(
               ADC_GRAY_INST,
               DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) {
        if (gray_sensor_elapsed_cycles(start, SysTick->VAL) >=
            timeout_cycles) {
            DL_ADC12_disableConversions(ADC_GRAY_INST);
            return false;   /* ADC 超时，硬件可能异常 */
        }
    }

    *value = DL_ADC12_getMemResult(ADC_GRAY_INST, DL_ADC12_MEM_IDX_0);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    return true;
}

/* ========================================================================== */
/*  初始化与原始采样                                                           */
/* ========================================================================== */

/**
 * @brief 初始化地址输出引脚和数字滞回状态。
 * @note 必须在 SYSCFG_DL_init() 后调用，确保 GPIO 已配置。
 */
void gray_sensor_init(void)
{
    DL_GPIO_clearPins(GPIO_GRAY_ADDRESS_PORT, GRAY_ADDRESS_PINS);
    g_digital_mask = 0xFFU;   /* 初始全白状态，避免首次滞回误判 */
}

/**
 * @brief 逐路扫描 8 路灰度传感器的原始 ADC 值。
 * @param raw 输出缓冲区，长度 GRAY_SENSOR_CHANNEL_COUNT(8)
 * @return true=所有 8 路转换均成功；false=任一路超时（整帧清零）
 *
 * @details 对每路：切换 74HC4051 地址 → 等待稳定 → 采样 8 次取平均。
 * 每路 8 次采样约 0.8ms，8 路共约 6.4ms，在 10ms 控制周期内可完成。
 * 任一路 ADC 超时则清零整帧并返回 false，调用方据此判断硬件故障。
 *
 * REVERSE_ORDER=1 时自动反转下标，使数组下标始终对应"左到右"物理顺序。
 */
bool gray_sensor_scan_raw(uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT])
{
    uint8_t channel;

    if (raw == NULL) {
        return false;
    }

    for (channel = 0U; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++) {
        uint32_t sum = 0U;
        uint8_t sample;

        /* 切换 74HC4051 到当前通道，等待模拟输出稳定 */
        gray_sensor_select_channel(channel);

        /* 每路采样 8 次取平均，抑制噪声 */
        for (sample = 0U;
             sample < GRAY_SENSOR_ADC_SAMPLES_PER_CHANNEL;
             sample++) {
            uint16_t adc_value = 0U;
            if (!gray_sensor_adc_read_once(&adc_value)) {
                /* 任一路 ADC 超时 → 整帧清零，service 报 ADC 故障 */
                memset(raw, 0, sizeof(uint16_t) * GRAY_SENSOR_CHANNEL_COUNT);
                return false;
            }
            sum += adc_value;
        }

#if GRAY_SENSOR_REVERSE_ORDER
        /* 传感器倒装时反转下标，使 [0] 始终对应最左通道 */
        raw[GRAY_SENSOR_CHANNEL_COUNT - 1U - channel] =
            (uint16_t)(sum / GRAY_SENSOR_ADC_SAMPLES_PER_CHANNEL);
#else
        raw[channel] =
            (uint16_t)(sum / GRAY_SENSOR_ADC_SAMPLES_PER_CHANNEL);
#endif
    }

    return true;
}

/* ========================================================================== */
/*  归一化与滞回二值化                                                         */
/* ========================================================================== */

/**
 * @brief 使用逐路白/黑标定值做归一化和滞回二值化。
 * @param raw                  原始 ADC 值数组（来自 scan_raw）
 * @param white                逐路白底标定值（反射强，ADC 值大）
 * @param black                逐路黑底标定值（反射弱，ADC 值小）
 * @param previous_digital_mask 上一帧二值掩码，用于滞回区保持状态
 * @param out                  输出：归一化值 + 二值掩码 + 有效性掩码
 * @return true=8 路全部有效(valid_mask==0xFF)；false=至少一路标定无效
 *
 * @details 本函数是灰度数据从"原始ADC"到"统一量纲"的核心转换：
 *
 * 1. 归一化（消除各路硬件差异）：
 *    将每路 raw 值按该路标定范围线性映射到 0~4095：
 *      raw ≤ black → normalized = 0（纯黑）
 *      raw ≥ white → normalized = 4095（纯白）
 *      black < raw < white → normalized = (raw - black) × 4095 / (white - black)
 *    归一化后各路可比，循迹算法的加权位置计算才有意义。
 *
 * 2. 有效性检查：
 *    span = white - black < MIN_CALIBRATION_SPAN(100) → 该路无效，
 *    valid_mask 对应位不置 1，service 检查 valid_mask != 0xFF 报 CAL 故障。
 *
 * 3. 滞回二值化（生成 digital_mask，供显示和调试）：
 *    上下阈值按标定值的 1/3 和 2/3 位置设置，中间为滞回区：
 *      gray_white = (black + 2×white) / 3   ← 上阈值（偏向白 2/3）
 *      gray_black = (2×black + white) / 3   ← 下阈值（偏向黑 2/3）
 *      raw ≥ gray_white → 判白（置 1）
 *      raw ≤ gray_black → 判黑（置 0）
 *      介于两者之间 → 保持上一帧状态（滞回，防抖动）
 *
 * @note 循迹算法使用 normalized（连续值）做加权位置计算，不用 digital_mask。
 */
bool gray_sensor_process(const uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT],
                         const uint16_t white[GRAY_SENSOR_CHANNEL_COUNT],
                         const uint16_t black[GRAY_SENSOR_CHANNEL_COUNT],
                         uint8_t previous_digital_mask,
                         gray_sensor_data_t *out)
{
    uint8_t channel;
    uint8_t digital = previous_digital_mask;
    uint8_t valid = 0U;

    if (raw == NULL || white == NULL || black == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    memcpy(out->raw, raw, sizeof(out->raw));

    for (channel = 0U; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++) {
        uint16_t white_value = white[channel];
        uint16_t black_value = black[channel];
        uint8_t bit = (uint8_t)(1U << channel);

        /* ── 有效性检查 ──
         * white ≤ black 说明标定逻辑错误（白底 ADC 应大于黑线 ADC）；
         * span < 100 说明该路几乎无法区分黑白（传感器坏/安装太高/光照不足）。
         * 无效通道 normalized 置 0，不设 valid 位。 */
        if (white_value <= black_value ||
            (uint16_t)(white_value - black_value) <
                GRAY_SENSOR_MIN_CALIBRATION_SPAN) {
            out->normalized[channel] = 0U;
            continue;
        }

        /* ── 归一化：raw → 0~4095 ──
         * 把各路不同的 ADC 动态范围统一映射到 0~4095，消除硬件差异。
         * 例：CH0 white=1742, black=80, span=1662
         *   raw=80  → normalized=0    （纯黑）
         *   raw=911 → normalized=2048 （半黑半白）
         *   raw=1742→ normalized=4095 （纯白） */
        uint32_t span = (uint32_t)white_value - black_value;
        uint32_t normalized;
        if (raw[channel] <= black_value) {
            normalized = 0U;          /* 比黑线还黑 → 0 */
        } else if (raw[channel] >= white_value) {
            normalized = GRAY_SENSOR_ADC_FULL_SCALE;  /* 比白底还白 → 4095 */
        } else {
            normalized = ((uint32_t)(raw[channel] - black_value) *
                          GRAY_SENSOR_ADC_FULL_SCALE) / span;
        }
        out->normalized[channel] = (uint16_t)normalized;
        valid |= bit;   /* 该路标定有效 */

        /* ── 滞回二值化：raw → digital_mask (1=白/0=黑) ──
         * 上下阈值取标定区间的 1/3 和 2/3 位置，中间 1/3 为滞回区。
         * 滞回区的通道保持上一帧状态，防止传感器在黑白边界处反复跳变。
         *
         *   black_value ─────── gray_black ─────── gray_white ─────── white_value
         *   ← 判黑(置0) →│← 滞回(保持) →│← 判白(置1) →
         *
         * 注意：循迹算法不用 digital_mask，它仅供 OLED 显示和调试。 */
        uint16_t gray_white = (uint16_t)(((uint32_t)black_value +
                                          2U * white_value) / 3U);
        uint16_t gray_black = (uint16_t)((2U * (uint32_t)black_value +
                                          white_value) / 3U);
        if (raw[channel] >= gray_white) {
            digital |= bit;           /* 足够白 → 置 1 */
        } else if (raw[channel] <= gray_black) {
            digital &= (uint8_t)~bit;  /* 足够黑 → 置 0 */
        }
        /* 介于 gray_black 和 gray_white 之间 → 保持上一帧状态（滞回） */
    }

    out->digital_mask = digital;   /* 更新后的二值掩码 */
    out->valid_mask = valid;       /* 8 位掩码：1=该路标定有效 */
    out->adc_ok = true;
    return (valid == 0xFFU);       /* 全部 8 路有效才返回 true */
}

/* ========================================================================== */
/*  对外便捷接口：采样 + 处理一步完成                                          */
/* ========================================================================== */

/**
 * @brief 使用 config 中固化的标定值完成一次扫描+归一化+二值化。
 * @param out 输出帧
 * @return true=扫描和处理均成功（8 路标定全有效）
 *
 * @details 这是 service 层每 10ms 调用的入口：
 *   1. scan_raw → 8 路 ADC 采样（约 6.4ms）
 *   2. process  → 归一化 + 滞回二值化（使用 g_default_white/black 标定值）
 *   3. 保存 digital_mask 供下一帧滞回使用
 *
 * ADC 超时 → out->adc_ok=false，service 报 ADC 故障；
 * 标定无效 → 返回 false，service 报 CALIBRATION 故障。
 */
bool gray_sensor_read(gray_sensor_data_t *out)
{
    uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT];

    if (out == NULL) {
        return false;
    }
    if (!gray_sensor_scan_raw(raw)) {
        /* ADC 采样失败（硬件超时）→ 清零输出，标记 ADC 异常 */
        memset(out, 0, sizeof(*out));
        out->adc_ok = false;
        return false;
    }

    /* 用固化标定值做归一化和滞回二值化 */
    bool valid = gray_sensor_process(raw, g_default_white, g_default_black,
                                     g_digital_mask, out);
    /* 保存本次 digital_mask 供下一帧的滞回判断使用 */
    g_digital_mask = out->digital_mask;
    return valid;
}

/* ========================================================================== */
/*  标定值只读访问器（供标定校验/调试使用）                                    */
/* ========================================================================== */

/** 获取当前固化的白底标定数组（只读），供标定质量校验。 */
const uint16_t *gray_sensor_default_white(void)
{
    return g_default_white;
}

/** 获取当前固化的黑底标定数组（只读），供标定质量校验。 */
const uint16_t *gray_sensor_default_black(void)
{
    return g_default_black;
}