/**
 * @file    maixcam.h
 * @brief   MaixCam 视觉数据接收模块接口 (UART3, PB2/PB3, 115200)
 *
 * @details MaixCam 通过 UART 发送 8 字节定长帧：
 *          帧头 0xFE + x(u16) + vx(i32, px/s) + 帧尾 0xFD（大端序）
 *          x=0xFFFF 表示当前没有真实检测到钢球。
 *
 *          本模块负责：
 *          - UART3 硬件初始化 + RX 中断接收
 *          - ISR 逐字节推入 FreeRTOS 队列
 *          - maixcam_parse_task 从队列阻塞读取，状态机解析帧
 *          - 对外提供 maixcam_get_ball() 读取最新解析结果
 */
#ifndef MAIXCAM_H
#define MAIXCAM_H

#include <stdint.h>
#include <stdbool.h>

/* ── MaixCam 串口帧定义（与 main.py uart_send_fast() 一致）── */
#define MAIXCAM_FRAME_HEAD  0xFE
#define MAIXCAM_FRAME_TAIL  0xFD
#define MAIXCAM_FRAME_LEN   8   /* 1+2+4+1 = 8 字节 */

/* ── 解析后的钢球检测数据 ── */
typedef struct {
    uint16_t x;         /**< 钢球中心 X 坐标（像素） */
    int32_t  vx_px_s;   /**< MaixCam 估计的 X 方向速度（像素/秒） */
    uint32_t frame_id;  /**< 帧序号（每收到一帧递增） */
    uint32_t tick_ms;   /**< MCU接收该帧时刻，用于拒绝陈旧位置 */
    bool     valid;     /**< 是否有有效数据 */
} maixcam_ball_t;

/**
 * @brief  初始化 UART3 + 创建接收队列
 * @note   在 SYSCFG_DL_init() 之后、调度器启动之前调用
 *         创建 FreeRTOS 队列，清空 RX FIFO，使能 RX 中断
 *         不创建解析任务（由 app_tasks_start 负责）
 */
void maixcam_init(void);

/**
 * @brief  向 MaixCam 发送图传开启指令 (0x5A 0x01 0xA5)
 * @note   阻塞发送 3 字节, 通过 UART3 TX (PB2) 发出
 */
void maixcam_send_image_transmission_start(void);
void maixcam_send_image_transmission_end(void);

/**
 * @brief  获取最新一帧钢球数据（非阻塞拷贝快照）
 * @param  out  输出缓冲
 * @return true=有有效数据；false=尚未收到任何完整帧
 */
bool maixcam_get_ball(maixcam_ball_t *out);

/**
 * @brief  MaixCam 串口数据解析任务入口
 * @param  arg  未使用
 * @note   从接收队列阻塞读取字节，运行帧状态机解析
 *         解析成功后更新全局 g_ball 结构体
 *         需通过 xTaskCreate() 创建
 */
void maixcam_parse_task(void *arg);

#endif /* MAIXCAM_H */