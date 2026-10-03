/**
 * @file    speed_service.h
 * @brief   速度环适配层接口：FreeRTOS 队列 + 编码器读取 + TB6612 输出。
 *
 * @details 本模块是 speed_loop_core 纯算法与硬件/RTOS 之间的适配层：
 *          - 管理速度目标/状态队列（模块私有，外部不接触句柄）；
 *          - 读取 encoder_get_data() 喂给算法核心；
 *          - 将核心输出的 PWM 写入 tb6612；
 *          - 对外暴露目标设置与只读状态快照。
 *          纯算法核心封装在 speed_loop_core 中，便于移植/单测。
 */
#ifndef SPEED_SERVICE_H
#define SPEED_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

/** 速度环控制周期 50ms（与 speed_loop_core 一致）。 */
#define SPEED_SERVICE_PERIOD_MS 50U

typedef enum {
    SPEED_PROFILE_TASK2_LAP = 0,
    SPEED_PROFILE_TASK4_COMBINED,
    SPEED_PROFILE_TASK5_LAP_BALL,
    SPEED_PROFILE_TASK6_SPECIFIED_BALL
} speed_service_profile_t;

/** 速度环对外状态快照；不含 PWM/PID 等内部状态。 */
typedef struct {
    int32_t left_rpm;         /**< 左轮实测速度 (RPM) */
    int32_t right_rpm;        /**< 右轮实测速度 (RPM) */
    int32_t left_target_rpm;  /**< 左轮目标速度 (RPM) */
    int32_t right_target_rpm; /**< 右轮目标速度 (RPM) */
    bool stopped;             /**< true=目标、斜坡和输出均已停稳 */
    int32_t left_count;       /**< 左轮累计脉冲（自 encoder_reset 以来） */
    int32_t right_count;      /**< 右轮累计脉冲（自 encoder_reset 以来） */
} speed_service_state_t;

/** 初始化速度环：创建队列、初始化核心与编码器，发布初始快照。成功返回 true。 */
bool speed_service_init(void);

/**
 * @brief 切换到指定菜单任务的独立 .speed 参数块。
 * @note 仅在 ESTOP 已锁存、车辆制动时允许调用；运行中由核心每个
 *       控制窗口重读参数，支持 Keil Watch 在线调整当前任务。
 */
bool speed_service_select_profile(speed_service_profile_t profile);

/** 设置左右轮目标速度 (RPM)，不启用低速前馈。 */
bool speed_service_set_target(int32_t left_rpm, int32_t right_rpm);

/** 设置左右轮目标速度，并指定是否启用 yaw 环专用低速前馈（应用层默认不用）。 */
bool speed_service_set_target_with_ff(int32_t left_rpm,
                                      int32_t right_rpm,
                                      bool low_speed_ff_enable);

/** 10ms 节拍入口：采样编码器，核心内部累计到 50ms 执行 PID 并输出 PWM。 */
void speed_service_step_10ms(void);

/** 只读取出最新速度状态快照；失败（队列未就绪/空）返回 false。 */
bool speed_service_get_state(speed_service_state_t *out);

/** 便捷查询：两轮是否已完全停稳。用于 yaw 到位等待判断。 */
bool speed_service_wheels_stopped_snapshot(void);

/** 锁存紧急停车：立即制动并拒绝后续非零目标，直到显式清除。 */
void speed_service_emergency_stop(void);

/** 清除紧急停车锁存；仅允许由新的显式运动命令调用。 */
void speed_service_clear_emergency_stop(void);

/** 查询紧急停车是否已锁存。 */
bool speed_service_is_emergency_stopped(void);

#endif /* SPEED_SERVICE_H */
