/**
 * @file    encoder.h
 * @brief   双路增量式编码器接口定义
 *
 * 编码器速度换算参数由 speed_service 按任务配置：
 *   - 官方任务2：2500 counts/rev
 *   - 官方任务4：1500 counts/rev
 *
 * 硬件映射 (实测):
 *   - 物理右轮: TIMG8 硬件 QEI (PA26=A, PA27=B)
 *   - 物理左轮: GPIO 双边沿软件解码 (PA25=A, PA14=B)
 *   - encoder_get_data() 内部做了左右交换，调用者无需关心硬件差异
 *
 * 使用示例:
 * @code
 *   encoder_init();                           // 初始化
 *   encoder_data_t data;
 *   encoder_get_data(&data);                  // 读取
 *   int32_t rpm = data.left_delta * ... ;     // 计算速度
 *   encoder_reset();                          // 零点复位
 * @endcode
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/**
 * @brief 编码器数据快照结构体
 *
 * 成员说明:
 *   - left_count / right_count: 累计计数（自初始化或上次 reset 以来的总脉冲数）
 *     正值 = 正转, 负值 = 反转
 *   - left_delta / right_delta: 自上次 encoder_get_data() 调用以来的脉冲增量
 *     用于速度计算 (delta / Δt / 每圈脉冲 × 60 = RPM)
 */
typedef struct {
    int32_t left_count;    /**< 左轮累计脉冲计数 */
    int32_t right_count;   /**< 右轮累计脉冲计数 */
    int32_t left_delta;    /**< 左轮增量脉冲 (两次读取之差) */
    int32_t right_delta;   /**< 右轮增量脉冲 (两次读取之差) */
} encoder_data_t;

/**
 * @brief  初始化编码器读取模块
 * @note   初始化左轮 QEI 计数器 + 右轮 GPIO 中断
 *         使能 GROUP1 中断用于 GPIO 软件正交解码
 *         应在 SYSCFG_DL_init() 之后调用
 */
void encoder_init(void);

/**
 * @brief  清零左右轮累计计数值
 * @note   通常在起跑/归零/切换控制模式时调用
 *         内部先停止计数器再清零, 保证原子性
 */
void encoder_reset(void);

/**
 * @brief  读取编码器累计计数和增量
 * @param  data  输出参数, 指向 encoder_data_t 的指针 (不可为 NULL)
 * @note   线程安全: 可在任务上下文中调用
 *         内部调用 update_left_count() 更新 QEI 累计值
 *         左右通道自动交换映射 (物理正确 → API 正确)
 */
void encoder_get_data(encoder_data_t *data);

/**
 * @brief  检查右轮编码器 GPIO 中断是否待处理
 * @return 非 0 = 有中断待处理, 0 = 无
 * @note   用于 GROUP1_IRQHandler 中断分发
 */
int encoder_right_int_is_pending(void);

/**
 * @brief  右轮编码器 GPIO 软件正交解码中断处理
 * @note   必须在 GROUP1_IRQHandler 中调用
 *         使用 16 项查表实现 4 倍频正交解码
 *         更新 volatile 全局变量 g_right_count
 */
void encoder_right_irq_handler(void);

#endif /* ENCODER_H */
