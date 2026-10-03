/**
 * @file    gray_sensor.h
 * @brief   八路灰度传感器驱动：地址切换、ADC 采样、归一化与滞回二值化。
 *
 * @details 传感器为无 MCU 基础款，内部 74HC4051 八选一模拟多路复用：
 *          PB0/PB1/PB10 输出地址 000~111 依次选择 CH1~CH8，PB25
 *          (ADC0_CH4) 采集模拟输出。每路需分别标定白底/黑底值，
 *          安装高度或光照变化后应重新标定。
 */
#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "gray_sensor/gray_sensor_config.h"

/** 一帧八路灰度采集与处理结果。
 *
 * 数据流：74HC4051 逐路选通 -> ADC 采样 8 次取平均 -> raw[]
 *        -> 标定归一化 -> normalized[] -> 滞回二值化 -> digital_mask
 *
 * 各字段含义：
 * - raw[]:        12 位 ADC 原始值（0~4095），未经标定修正
 * - normalized[]: 归一化值（0~4095），0=纯黑/4095=纯白，已消除各路差异
 * - digital_mask: 滞回二值掩码，bit=1 白底/bit=0 黑线，中间区保持上次
 * - valid_mask:   标定有效性掩码，bit=1 表示该路 white-black>=100
 * - adc_ok:       true=本帧 8 路 ADC 转换均未超时
 *
 * 循迹算法使用 normalized[] 做加权位置计算，不用 digital_mask。 */
typedef struct {
    uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT];        /**< 原始 ADC 值 */
    uint16_t normalized[GRAY_SENSOR_CHANNEL_COUNT]; /**< 归一化值（0~4095） */
    uint8_t digital_mask;     /**< 1=白底，0=黑线，中间滞回区保持上次状态 */
    uint8_t valid_mask;       /**< 对应位为 1 表示该通道黑白标定跨度有效 */
    bool adc_ok;              /**< true=本帧所有通道 ADC 转换未超时 */
} gray_sensor_data_t;

/**
 * @brief  初始化地址输出引脚和数字滞回状态。
 * @note   必须在 SYSCFG_DL_init() 后调用。
 */
void gray_sensor_init(void);

/**
 * @brief  仅扫描八路原始 ADC 值。
 * @param  raw  输出缓冲区，长度 GRAY_SENSOR_CHANNEL_COUNT
 * @return true=所有通道转换成功；false=任一通道超时
 */
bool gray_sensor_scan_raw(uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT]);

/**
 * @brief  使用指定逐路白/黑标定值做归一化和滞回二值化。
 * @param  raw                  原始 ADC 值
 * @param  white                逐路白底标定值
 * @param  black                逐路黑底标定值
 * @param  previous_digital_mask 上一帧二值结果，用于中间滞回区保持状态
 * @param  out                  输出：归一化值与二值掩码
 * @return true=处理成功
 */
bool gray_sensor_process(const uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT],
                         const uint16_t white[GRAY_SENSOR_CHANNEL_COUNT],
                         const uint16_t black[GRAY_SENSOR_CHANNEL_COUNT],
                         uint8_t previous_digital_mask,
                         gray_sensor_data_t *out);

/**
 * @brief  使用 gray_sensor_config.h 中固化的标定值完成一次扫描和处理。
 * @param  out  输出帧
 * @return true=扫描与处理均成功
 */
bool gray_sensor_read(gray_sensor_data_t *out);

/** 获取当前固化的白底标定数组（只读）。 */
const uint16_t *gray_sensor_default_white(void);

/** 获取当前固化的黑底标定数组（只读）。 */
const uint16_t *gray_sensor_default_black(void);

#endif /* GRAY_SENSOR_H */
