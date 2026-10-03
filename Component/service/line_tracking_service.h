/**
 * @file line_tracking_service.h
 * @brief 循迹service：负责RTOS/硬件编排并发布控制与安全状态。
 *
 * @details 本层是纯control与硬件服务之间的唯一适配边界：读取灰度和
 * 编码器状态，调用control，向speed_service下发最终左右轮目标，并监督
 * 定距/超时/故障。上层只能通过命令队列和只读快照访问，不直接操作
 * control内部状态。
 */
#ifndef LINE_TRACKING_SERVICE_H
#define LINE_TRACKING_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "gray_sensor/gray_sensor.h"
#include "control/line_tracking_control.h"

/** 会终止本次循迹的故障原因；除NONE外均锁存速度服务ESTOP。 */
typedef enum {
    LINE_TRACK_FAULT_NONE = 0,    /**< 无故障，包含正常到达和用户中止 */
    LINE_TRACK_FAULT_ADC,         /**< 任一灰度ADC转换失败/超时 */
    LINE_TRACK_FAULT_CALIBRATION, /**< 标定未就绪、通道掩码或灰度帧无效 */
    LINE_TRACK_FAULT_LOST,        /**< 初始无线或连续丢线超过搜索时限 */
    LINE_TRACK_FAULT_SPEED_STATE, /**< 速度快照/目标队列不可用或下发失败 */
    LINE_TRACK_FAULT_PROGRESS     /**< 无编码器进度或单次运行超时 */
} line_tracking_fault_t;

/**
 * @brief 供app_tasks、OLED和UART只读的原子状态快照。
 * @note service使用长度1覆盖队列发布，读取者得到同一10ms周期的一致字段。
 * inactive时控制输出字段为0，但灰度强度/判线字段仍持续实时更新。
 */
typedef struct {
    int32_t target_counts;    /**< 本次目标编码器计数 */
    int32_t travelled_counts;  /**< 已行驶编码器计数 */
    int32_t remaining_counts;  /**< 剩余编码器计数 */
    int32_t line_position;     /**< 黑线加权位置（左负右正，约 ±7×1024） */
    int32_t steering_error;    /**< PD误差，等于滤波位置的相反数 */
    int32_t base_rpm;          /**< control弯道调速后的当前基础速度 */
    int32_t turn_rpm;          /**< 平滑后的循迹外环差速修正 */
    int32_t requested_turn_rpm; /**< 重捕blend/通用slew之前的请求差速 */
    uint32_t line_strength;      /**< 八路黑色强度总和 */
    uint16_t line_peak_strength; /**< 单路最大黑色强度，无滞回 */
    uint8_t strong_channel_count; /**< 明确黑色通道数，无滞回 */
    uint8_t digital_mask;        /**< 八路二值掩码：1=白底，0=黑线 */
    bool output_valid;         /**< true=本周期控制输出已成功下发 */
    bool active;               /**< true=循迹运行中 */
    bool reached;              /**< true=已到达目标距离并制动 */
    bool aborted;              /**< true=用户主动中止 */
    bool sensor_valid;         /**< true=本帧传感器与标定均有效 */
    bool line_detected;        /**< true=本帧强黑通道为1~4路 */
    bool checkpoint_detected;    /**< true=本帧检测到全黑检查点 */
    bool reacquire_blending;   /**< true=重捕后的差速正在平滑 */
    uint8_t checkpoint_count;    /**< 已遇到全黑检查点的次数 */
    line_tracking_fault_t fault; /**< 当前故障码 */
    uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT];        /**< 原始 ADC 值 */
    uint16_t normalized[GRAY_SENSOR_CHANNEL_COUNT]; /**< 归一化值 */
} line_tracking_service_state_t;

/**
 * @brief 初始化循线服务：创建命令/状态队列，复位控制器与距离监督。
 * @return true=两个队列均创建成功并已发布初始停车快照
 * @note 调度器启动前调用一次；失败时上层必须保持电机制动。
 */
bool line_tracking_service_init(void);

/** 停车状态下切换到对应菜单任务的独立 .line 参数块。 */
bool line_tracking_service_select_profile(line_tracking_profile_t profile);

/**
 * @brief  启动定距循迹：仅接收目标编码器计数和基础轮速。
 * @param  target_counts 目标编码器计数，必须大于 0
 * @param  base_rpm      两轮公共基础速度，范围见line_tracking_config.h
 * @return true=IDLE且START命令已写入长度1命令队列
 * @note 只负责参数初检和投递；灰度/速度/标定检查在下个10ms周期执行。
 *       运行中重复START返回false，避免重置control历史导致目标突降。
 */
bool line_tracking_service_start(int32_t target_counts,
                                 int32_t base_rpm);

/**
 * @brief 启动检查点循迹，不使用编码器距离作为终点。
 * @param base_rpm 两轮公共基础速度
 * @note 编码器仅用于卡死保护和行程遥测；检查点由上层确认并请求中止。
 */
bool line_tracking_service_start_checkpoint(int32_t base_rpm);

/**
 * @brief 设置本次循迹的基础速度上限，0表示关闭上限。
 * @note 仅限制纵向速度并同比缩放差速，不改变检查点/定距终点条件。
 */
bool line_tracking_service_set_speed_cap(int32_t cap_rpm);

/**
 * @brief 请求中止当前循迹。
 * @return true=ABORT命令已写入队列
 * @note service在下一个10ms周期消费；紧急情况应同时直接锁存ESTOP。
 */
bool line_tracking_service_abort(void);

/**
 * @brief 10ms入口：采集灰度、调用control并向速度服务下发左右目标。
 * @note 必须严格每10ms调用一次。停止状态仍执行无状态灰度预览，但不会
 *       推进PD/丢线历史，也不会持续写(0,0)覆盖其他运动模式。
 */
void line_tracking_service_step_10ms(void);

/**
 * @brief 读取最新循迹状态快照，不消费队列内容。
 * @param  out  输出缓冲区，非空
 * @return true=成功读取
 */
bool line_tracking_service_get_state(line_tracking_service_state_t *out);

/**
 * @brief  从状态快照判断是否正在循迹。
 * @return true=循迹运行中
 */
bool line_tracking_service_is_active_snapshot(void);

#endif /* LINE_TRACKING_SERVICE_H */
