/**
 * @file line_tracking_config.h
 * @brief 循线算法中真正跨任务共享的传感器几何常量。
 *
 * @details
 * Task2/4/5/6 的 PD、丢线、弯速、轮速整形、定距和安全参数已经分别
 * 移到 Component/config/task_params.c。这里仅保留由同一块八路灰度板
 * 物理结构决定、不能按任务分别调节的常量。
 *
 * 灰度黑白 ADC 标定仍在 gray_sensor_config.h；换传感器安装高度、赛道
 * 或环境光后应重新标定该文件，而不是修改任务参数来掩盖采样错误。
 */
#ifndef LINE_TRACKING_CONFIG_H
#define LINE_TRACKING_CONFIG_H

/** service公开START接口的整车安全上限，所有任务都不得超过。 */
#define LINE_TRACKING_MAX_BASE_RPM 250

/** 八路传感器从左到右的空间权重；倒装应先改灰度通道顺序。 */
#define LINE_TRACKING_POSITION_WEIGHTS {-7, -5, -3, -1, 1, 3, 5, 7}

/** 加权位置的整数放大倍数；修改后所有任务的位置阈值均需重标。 */
#define LINE_TRACKING_POSITION_SCALE 1024

#endif /* LINE_TRACKING_CONFIG_H */
