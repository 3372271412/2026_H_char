/**
 * @file task_params.h
 * @brief 六任务参数的统一声明入口。
 *
 * @details
 * 本文件只定义参数结构和全局参数块；所有默认值集中在 task_params.c。
 * 参数分成两类：
 *
 * 1. g_hardware_calibration：整车共享的物理标定。修改后会影响所有管球任务，
 *    只应在换车、重装机构或重新机械归零时调整。
 * 2. g_task1_params ~ g_task6_params：菜单任务独立参数。调节某一参数块不会
 *    改写其他任务的参数，循线、轮速和管球控制也分别保存独立副本。
 *
 * 所有参数块均为 volatile 全局变量，可直接加入 Keil Watch。例如：
 *   g_task4_params.ball.kp_pos_milli
 *   g_task5_params.line.kd_milli
 *   g_task6_params.lap.speed.counts_per_rev
 *
 * @warning Keil Watch 修改仅在本次上电期间有效，复位后恢复 task_params.c
 *          中的编译默认值。运行中修改参数应小步进行；机械方向、零点与
 *          硬限位只能在停车且抬起驱动轮时修改。
 */
#ifndef TASK_PARAMS_H
#define TASK_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

/** 每个任务独立保持表的固定点数。X 必须从小到大排列。 */
#define TASK_HOLD_CAL_POINT_COUNT 5U

/**
 * @brief 全任务共享的管球机构物理标定。
 * @note 这些值描述同一套硬件，不属于任何单项任务，因此有意共享。
 */
typedef struct {
    uint8_t stepper_addr;             /**< X42S 总线地址；必须与驱动器拨号/配置一致。 */
    int32_t motor_zero_offset_pulses; /**< 机械水平零点微调，单位脉冲；大偏差应机械归零。 */
    int32_t tilt_sign;                /**< 相机误差到倾角方向：只允许 +1 或 -1。 */
    int32_t hard_max_tilt_pos;        /**< 机构正方向绝对安全限位，单位脉冲。 */
    int32_t hard_max_tilt_neg;        /**< 机构负方向绝对安全限位的绝对值，单位脉冲。 */
    uint32_t stepper_command_gap_ms;  /**< X42S连续配置命令之间的等待时间。 */
} hardware_calibration_t;

/**
 * @brief 单个任务的管球闭环参数。
 * @note Task3/4/5/6 各有完整独立副本，不再复用旧 g_t2_* 参数。
 */
typedef struct {
    /* 目标、级联控制与任务内安全限幅。 */
    int32_t default_target_x;         /**< 默认球目标 X，单位相机像素。 */
    int32_t kp_pos_milli;             /**< 位置外环 Kp×1000；误差→期望球速。 */
    int32_t kp_vel_milli;             /**< 速度内环 Kp×1000；速度误差→倾角脉冲。 */
    int32_t max_desired_vel_px_s;     /**< 期望球速绝对上限，单位 px/s。 */
    int32_t max_tilt_pos;             /**< 本任务正向倾角上限，不得超过硬限位。 */
    int32_t max_tilt_neg;             /**< 本任务负向倾角上限绝对值。 */

    /* 摩擦死区与静止启动。 */
    int32_t dead_zone_pulses;         /**< 线性输出小于该值时允许死区补偿。 */
    int32_t dead_boost_pulses;        /**< 通用死区补偿倾角，单位脉冲。 */
    int32_t pos_static_tilt;          /**< 向高 X 方向克服静摩擦的最大启动倾角。 */
    int32_t dead_vel_px_s;            /**< |球速|低于此值才视为静止。 */
    uint32_t pos_kick_far_ms;         /**< 远距离正向启动脉冲持续时间。 */
    uint32_t pos_kick_near_ms;        /**< 近距离正向启动脉冲持续时间。 */
    int32_t pos_kick_near_tilt;       /**< 近距离正向启动脉冲幅值。 */
    int32_t pos_kick_far_error_px;    /**< 正向误差达到该值时使用远距离启动策略。 */
    uint32_t pos_kick_cooldown_ms;    /**< 两次启动脉冲之间的最短冷却时间。 */

    /* 到达判定、视觉滤波和输出整形。 */
    int32_t tolerance_px;             /**< |位置误差|小于该值视为进入目标区。 */
    int32_t reached_vel_px_s;         /**< 到位时允许的最大绝对球速。 */
    uint32_t stable_time_ms;          /**< 连续满足到位条件多久才置 reached。 */
    int32_t vel_filter_alpha_milli;   /**< 球速低通新值权重×1000，0最慢，1000不过滤。 */
    int32_t max_input_vel_px_s;       /**< 相机球速输入绝对限幅，抑制异常帧尖峰。 */
    int32_t output_slew_pulses;       /**< 每帧倾角最大变化；0 表示关闭变化率限制。 */

    /* 丢球处理。 */
    uint32_t lost_timeout_ms;         /**< 无新视觉帧超过该时间后进入 LOST。 */
    int32_t edge_low_x;               /**< 左侧画面边缘阈值。 */
    int32_t edge_high_x;              /**< 右侧画面边缘阈值。 */
    int32_t edge_recover_tilt;        /**< 边缘丢球时反向回收倾角绝对值。 */
    uint32_t edge_recover_ms;         /**< 边缘回收倾角最长保持时间。 */
    uint32_t homing_time_ms;          /**< 回水平后等待多久再进入闭环。 */

    /* 高速倾角自适应：球速越高，允许继续加速的正倾角越小。 */
    int32_t tilt_reduce_start_vel;    /**< 超过此绝对球速才开始缩小正倾角。 */
    int32_t tilt_reduce_vel_span;     /**< 再增加该速度时完成一次线性缩减。 */
    int32_t min_tilt_divisor;         /**< 最小正倾角=max_tilt_pos/该值，须大于0。 */

    /* 行车管球专用：Task5/6 行驶中抑制极限环。 */
    int32_t drive_hold_tolerance_px;  /**< 行驶中进入静态保持表的更小容差。 */
    int32_t drive_kp_pos_milli;       /**< 行车阶段位置Kp×1000；0=沿用静止Kp。 */
    int32_t drive_kp_vel_milli;       /**< 行车阶段速度Kp×1000；0=沿用静止Kp。 */
    int32_t drive_vel_alpha_milli;    /**< 行车阶段球速滤波系数；负数=沿用静止值。 */
    int32_t error_boost_start_px;     /**< 超过该位置误差才增加连续纠偏。 */
    int32_t error_boost_milli;        /**< 每超出1px增加的倾角×1000。 */

    /* 静态保持倾角。enabled=false 时使用下方线性公式。 */
    int32_t hold_center_x;            /**< 线性保持公式的中心 X。 */
    int32_t hold_gain_milli;          /**< 保持倾角/像素×1000。 */
    int32_t hold_cal_x[TASK_HOLD_CAL_POINT_COUNT];       /**< 标定 X，严格递增。 */
    int32_t hold_cal_tilt[TASK_HOLD_CAL_POINT_COUNT];    /**< 对应静态保持倾角。 */
    uint8_t hold_cal_count;           /**< 有效点数，范围0~5，至少2点才插值。 */
    bool hold_cal_enabled;            /**< true=启用本任务独立保持表。 */

    /* X42S 每次位置命令的运动参数。 */
    uint16_t motor_vel_rpm;           /**< 步进电机位置模式速度。 */
    uint8_t motor_acc;                /**< 步进电机位置模式加速度档位。 */
    uint8_t motor_raf;                /**< 位置模式 RAF 参数，通常保持1。 */
} ball_control_params_t;

/**
 * @brief 单个菜单任务的循线参数。
 * @note 多数控制参数在每次10ms更新时读取；turn限幅比例和检查点自动
 *       停车次数在任务START时锁存，修改后需停车并重新启动。
 */
typedef struct {
    /* 转向 PD。所有 milli 增益均为真实值×1000。 */
    int32_t kp_high_milli;            /**< 大偏差比例增益。 */
    int32_t kp_low_milli;             /**< 小偏差比例增益。 */
    int32_t kp_switch_threshold;      /**< 进入高 Kp 的误差阈值；0=始终高 Kp。 */
    int32_t kp_release_threshold;     /**< 从高 Kp 退回低 Kp 的阈值。 */
    int32_t ki_milli;                 /**< 积分增益；循线通常为0。 */
    int32_t kd_milli;                 /**< 微分增益；增大可抑制摆动但放大噪声。 */
    int32_t deadband;                 /**< 中心位置死区；0=关闭。 */
    int32_t position_filter_cutoff;   /**< 小偏差低通范围；0=关闭。 */
    int32_t position_filter_old_weight; /**< 低通旧值权重，须>=0。 */
    uint8_t line_strong_channel_max; /**< 1~此通道数强黑视为普通线。 */
    uint8_t checkpoint_channel_min;  /**< 达到该强黑通道数视为横向检查点。 */
    uint8_t checkpoint_stop_count;   /**< 定距模式第几次检查点自动停；0=不按检查点停。 */

    /* 差速限幅、丢线搜索与重捕。 */
    int32_t turn_slew_up_rpm;         /**< turn增大时每10ms最大变化；0=不限。 */
    int32_t turn_slew_down_rpm;       /**< turn回零/反向时最大变化；0=不限。 */
    int32_t turn_limit_numerator;     /**< turn限幅比例分子。 */
    int32_t turn_limit_denominator;   /**< turn限幅比例分母，必须>0。 */
    int32_t turn_limit_initial_rpm;   /**< 尚未 START 时 PID 临时输出限幅。 */
    int32_t search_direction_threshold; /**< 记录最后黑线方向的最小位置量。 */
    int32_t lost_search_turn_percent;/**< 丢线搜索turn占基础速度百分比。 */
    uint16_t lost_search_ticks;       /**< 允许连续搜索的10ms周期数。 */
    int32_t direction_release_position; /**< 黑线回到该中心区才允许解除方向锁存。 */
    uint16_t direction_release_ticks; /**< 解除方向锁存的连续确认周期。 */
    int32_t reacquire_blend_step_rpm; /**< 重捕时 turn 每周期最大变化。 */
    uint16_t reacquire_blend_ticks;   /**< 重捕平滑持续周期。 */

    /* 弯道速度规划与轮速整形。 */
    int32_t corner_speed_percent;     /**< 深弯速度占直线基础速度百分比。 */
    int32_t corner_speed_max_rpm;     /**< 深弯基础速度绝对上限。 */
    int32_t speed_full_position;      /**< 小于该|位置|保持全速。 */
    int32_t speed_corner_position;    /**< 大于该|位置|使用深弯速度。 */
    uint16_t corner_recovery_ticks;   /**< 重捕后继续保持弯速的周期数。 */
    int32_t base_accel_rpm_per_tick;  /**< 出弯基础速度每10ms恢复步长。 */
    int32_t base_decel_rpm_at_ref;    /**< 参考速度下的入弯减速步长。 */
    int32_t base_decel_ref_rpm;       /**< 入弯减速缩放参考速度，必须>0。 */
    uint16_t startup_ticks;           /**< 启动低速保护持续10ms周期数。 */
    int32_t startup_base_rpm;         /**< 启动保护期间基础速度上限。 */
    int32_t wheel_accel_rpm_per_tick; /**< 单轮目标绝对值增加步长。 */
    uint16_t wheel_accel_interval_ticks; /**< 单轮加速整形分频，至少1。 */

    /* service层安全、定距和减速。 */
    int32_t brake_margin_counts;      /**< 定距任务提前制动计数。 */
    int32_t distance_decel_counts;    /**< 距制动点多远开始平方根降速。 */
    int32_t distance_decel_coefficient; /**< 平方根降速系数。 */
    int32_t distance_min_rpm;         /**< 定距末段最低基础速度。 */
    uint16_t no_progress_limit_ticks; /**< 无编码器进度故障周期数。 */
    uint32_t max_run_ticks;           /**< 单次循线最长10ms周期数。 */
} line_control_params_t;

/** 单个任务的左右轮速度内环参数。 */
typedef struct {
    int32_t kp_milli;                 /**< 增量式速度 PID 比例增益×1000。 */
    int32_t ki_milli;                 /**< 增量式速度 PID 积分增益×1000。 */
    int32_t kd_milli;                 /**< 增量式速度 PID 微分增益×1000。 */
    int32_t pwm_min;                  /**< PID PWM输出下限；底层仍强制不小于-100。 */
    int32_t pwm_max;                  /**< PID PWM输出上限；底层仍强制不大于+100。 */
    int32_t ramp_step_rpm;            /**< 每个50ms速度设定点最大变化。 */
    int32_t rpm_filter_alpha_milli;   /**< 实测RPM低通新值权重×1000。 */
    int32_t start_ff_pwm;             /**< 低速克服静摩擦的前馈PWM。 */
    int32_t start_ff_max_setpoint_rpm;/**< 高于此设定点禁用低速前馈。 */
    int32_t start_ff_error_rpm;       /**< 速度误差不超过此值时禁用前馈。 */
    int32_t start_ff_min_setpoint_rpm;/**< 低于此设定点禁用前馈。 */
    int32_t start_ff_full_setpoint_rpm; /**< 前馈线性缩放的满幅设定点。 */
    int32_t start_ff_min_pwm;         /**< 低设定点时允许的最小前馈PWM。 */
    uint32_t counts_per_rev;          /**< 输出轴每圈编码器计数，换轮/减速比后重标。 */
} speed_control_params_t;

/** 菜单 Task1：仅图传启停。 */
typedef struct {
    uint32_t stream_close_delay_ms;   /**< 按停后延迟关闭图传的时间。 */
} task1_params_t;

/** 菜单 Task2：纯循线一圈。 */
typedef struct {
    uint32_t stream_close_delay_ms;   /**< 完成后图传延迟关闭时间。 */
    int32_t base_rpm;                 /**< 一圈循线直线基础速度。 */
    int32_t virtual_target_counts;    /**< 不作为终点的超大定距值。 */
    uint8_t line_detect_frames;       /**< 终点横线连续确认帧数。 */
    uint8_t line_detect_strong;       /**< 判为横线的最少强黑通道数。 */
    uint8_t line_clear_frames;        /**< 驶离起点横线的连续确认帧数。 */
    uint16_t start_grace_ticks;       /**< 启动后忽略起点横线的10ms周期数。 */
    line_control_params_t line;       /**< Task2独立循线参数。 */
    speed_control_params_t speed;     /**< Task2独立轮速参数。 */
} task2_params_t;

/** 菜单 Task3：O→+5cm→-5cm。 */
typedef struct {
    uint32_t stream_close_delay_ms;   /**< 完成后图传延迟关闭时间。 */
    int32_t target_pos5_x;            /**< 物理+5cm对应相机X。 */
    int32_t target_neg5_x;            /**< 物理-5cm对应相机X。 */
    int32_t arrive_threshold_px;      /**< 分段目标到达像素阈值。 */
    uint32_t arrive_stable_ms;        /**< 最终-5cm到达稳定时间。 */
    uint32_t pos_arrive_stable_ms;    /**< +5cm到达稳定时间。 */
    int32_t final_max_vel_px_s;       /**< 最终到达允许的最大绝对球速。 */
    uint32_t hold_pos5_ms;            /**< +5cm处停留时间，0=立即折返。 */
    int32_t pos_brake_margin_px;      /**< +5cm前提前切换目标的基础裕量。 */
    int32_t pos_brake_speed_factor;   /**< 速度自适应预制动系数×100。 */
    ball_control_params_t ball;       /**< Task3独立管球参数。 */
} task3_params_t;

/** 菜单 Task4：A到B定距循线并保持球在中心。 */
typedef struct {
    uint32_t stream_close_delay_ms;
    int32_t target_counts;            /**< A到B目标编码器计数。 */
    int32_t base_rpm;                 /**< A到B基础轮速。 */
    int32_t ball_target_x;            /**< 本任务球中心目标X。 */
    uint32_t timeout_ms;              /**< 发车后运行超时。 */
    int32_t start_ff;                 /**< 车辆起步时管球倾角前馈。 */
    int32_t end_ff;                   /**< 车辆制动时管球倾角前馈。 */
    uint32_t ball_lost_stop_ms;       /**< 连续丢球多久后故障停车。 */
    uint32_t prepare_stable_ms;       /**< 发车前球居中稳定时间。 */
    uint32_t prepare_timeout_ms;      /**< 发车前居中超时。 */
    uint32_t brake_stable_ms;         /**< 轮速停稳连续确认时间。 */
    uint32_t brake_timeout_ms;        /**< 制动状态最长时间。 */
    uint32_t hold_stable_ms;          /**< 停车后球居中确认时间。 */
    uint32_t hold_timeout_ms;         /**< 停车后球居中超时。 */
    uint32_t start_ff_duration_ms;    /**< 起步前馈持续时间。 */
    int32_t end_ff_distance_counts;   /**< 距终点该计数时使用完整制动前馈。 */
    int32_t end_ff_ramp_counts;       /**< 制动前馈线性渐增区长度。 */
    int32_t wheel_stop_rpm;           /**< 轮速绝对值低于此值视为停稳。 */
    ball_control_params_t ball;
    line_control_params_t line;
    speed_control_params_t speed;
} task4_params_t;

/** Task5/6共用的“一圈+管球”状态机参数结构，但两个全局实例完全独立。
 * @note 顶层状态机字段在按键START时统一锁存；ball/line/speed控制参数
 *       仍由对应控制周期读取。修改顶层字段后应停止并重跑任务。 */
typedef struct {
    uint32_t stream_close_delay_ms;
    int32_t base_rpm;                 /**< 一圈基础轮速。 */
    int32_t ball_target_x;            /**< 固定目标X；Task6运行时会锁定现场坐标。 */
    uint32_t timeout_ms;              /**< 整个任务超时。 */
    int32_t start_ff;                 /**< 起步管球前馈。 */
    int32_t brake_ff;                 /**< 停车管球前馈。 */
    int32_t corner_ff_milli;          /**< 弯道前馈强度，历史单位脉冲/RPM×1000。 */
    int32_t lap_estimate_counts;      /**< 一圈编码器估计，成功后可在线更新。 */
    int32_t decel_counts;             /**< 末段降速区长度。 */
    int32_t end_min_rpm;              /**< 末段最低轮速。 */
    uint32_t ball_lost_stop_ms;
    uint32_t prepare_stable_ms;
    uint32_t prepare_timeout_ms;
    uint32_t checkpoint_clear_ms;     /**< 驶离起点横线确认时间。 */
    uint32_t checkpoint_confirm_ms;   /**< 再次检测终点横线确认时间。 */
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
    int32_t brake_start_min_rpm;      /**< 制动斜坡起始速度的最小值，避免直接跳零。 */
    uint32_t brake_ramp_ms;
    uint32_t brake_stable_ms;
    uint32_t brake_timeout_ms;
    uint32_t hold_stable_ms;
    uint32_t hold_timeout_ms;
    int32_t wheel_stop_rpm;
    int32_t decel_min_counts;          /**< 小于该长度时禁用末段降速。 */
    int32_t lap_learn_min_counts;      /**< 合理一圈计数下限。 */
    int32_t lap_learn_max_counts;      /**< 合理一圈计数上限。 */
    ball_control_params_t ball;
    line_control_params_t line;
    speed_control_params_t speed;
} lap_ball_params_t;

/** 菜单 Task6 在 Task5型一圈参数之外的动态目标参数。 */
typedef struct {
    lap_ball_params_t lap;
    uint32_t ball_fresh_max_ms;       /**< 按键锁定目标时允许的最大视觉帧年龄。 */
    int32_t auto_target_min_x;        /**< 允许锁定的最小相机X。 */
    int32_t auto_target_max_x;        /**< 允许锁定的最大相机X。 */
    int32_t ff_low_x;                 /**< 动态启停前馈插值低端X。 */
    int32_t ff_high_x;                /**< 动态启停前馈插值高端X。 */
    int32_t start_ff_low;             /**< 低端X起步前馈。 */
    int32_t start_ff_high;            /**< 高端X起步前馈。 */
    int32_t brake_ff_low;             /**< 低端X停车前馈。 */
    int32_t brake_ff_high;            /**< 高端X停车前馈。 */
} task6_params_t;

extern volatile hardware_calibration_t g_hardware_calibration;
extern volatile task1_params_t g_task1_params;
extern volatile task2_params_t g_task2_params;
extern volatile task3_params_t g_task3_params;
extern volatile task4_params_t g_task4_params;
extern volatile lap_ball_params_t g_task5_params;
extern volatile task6_params_t g_task6_params;

#endif /* TASK_PARAMS_H */
