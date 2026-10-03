/**
 * @file    maixcam.c
 * @brief   MaixCam 视觉数据接收模块 (UART3, PB2/PB3, 115200)
 * @note
 *   ── 硬件配置 ──
 *   UART3, PB2=TX, PB3=RX, 115200 8N1
 *   RX 中断优先级: 最低 (不抢占控制相关中断)
 *
 *   ── 数据流 ──
 *   MaixCam main.py → UART TX → MSPM0 PB3 (UART3 RX)
 *     ISR: 读字节 → xQueueSendFromISR → FreeRTOS 队列
 *     maixcam_parse_task: xQueueReceive 阻塞读 → 帧状态机解析
 *       → 大端转小端 → 更新 g_ball 全局结构体
 *     oled_task: maixcam_get_ball() 读快照 → OLED 显示
 *
 *   ── 帧格式（大端，共 8 字节）──
 *   帧头 0xFE(1B) + x(uint16) + vx(int32, px/s) + 帧尾 0xFD(1B)
 *   MSPM0G3507 (Cortex-M0+) 为小端架构，需做字节序转换
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "maixcam/maixcam.h"
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  配置常量
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief 接收队列长度 (字节数) */
#define MAIXCAM_RX_QUEUE_LEN  64

/* ═══════════════════════════════════════════════════════════════════════════
 *  FreeRTOS 同步对象
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief 接收字节队列: ISR 写入, parse_task 读取 */
static QueueHandle_t xRxQueue = NULL;

/* ═══════════════════════════════════════════════════════════════════════════
 *  全局解析结果
 *  parse_task 写, oled_task 读; 用 taskENTER_CRITICAL 保证原子拷贝
 * ═══════════════════════════════════════════════════════════════════════════ */

static volatile maixcam_ball_t g_ball = {0};

/* ═══════════════════════════════════════════════════════════════════════════
 *  大端转小端工具函数
 *  MaixCam (Python struct.pack ">") 发送大端数据,
 *  MSPM0G3507 (Cortex-M0+) 为小端, 需手动转换
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  大端 2 字节 → uint16
 */
static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/**
 * @brief  大端 4 字节 → int32
 */
static inline int32_t be32s(const uint8_t *p)
{
    uint32_t value = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
    return (int32_t)value;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════════════════════════════════════════ */

void maixcam_init(void)
{
    /* ── 创建接收字节队列 ── */
    xRxQueue = xQueueCreate(MAIXCAM_RX_QUEUE_LEN, sizeof(uint8_t));

    /* ── 清空 RX FIFO 残留 ──
     * 上电/复位时 UART 引脚可能产生虚假边沿, 清空避免收到无效字节 */
    uint8_t dummy;
    while (DL_UART_receiveDataCheck(UART_3_INST, &dummy)) {}

    /* ── 使能 RX 中断 ──
     * 优先级: configLIBRARY_LOWEST_INTERRUPT_PRIORITY (最低)
     * 原因: 视觉数据接收不参与实时控制, 不应抢占 TIMG0/GROUP1 */
    NVIC_SetPriority(UART_3_INST_INT_IRQN, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
    DL_UART_enableInterrupt(UART_3_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  发送: 图传开启指令 0x5A 0x01 0xA5
 * ═══════════════════════════════════════════════════════════════════════════ */

void maixcam_send_image_transmission_start(void)
{
    static const uint8_t cmd[3] = {0x5A, 0x01, 0xA5};
    for (int i = 0; i < 3; i++) {
        DL_UART_Main_transmitDataBlocking(UART_3_INST, cmd[i]);
    }
}


void maixcam_send_image_transmission_end(void)
{
    static const uint8_t cmd[3] = {0x5A, 0x02, 0xA5};
    for (int i = 0; i < 3; i++) {
        DL_UART_Main_transmitDataBlocking(UART_3_INST, cmd[i]);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ISR: UART3 接收中断
 *  ISR只做"读字节 → 推队列", 不在中断中解析
 * ═══════════════════════════════════════════════════════════════════════════ */

void UART_3_INST_IRQHandler(void)
{
    DL_UART_IIDX iid = DL_UART_getPendingInterrupt(UART_3_INST);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (iid == DL_UART_IIDX_RX) {
        uint8_t byte = DL_UART_receiveData(UART_3_INST);

        /* 推入接收队列 (FROM_ISR 版本, 不阻塞) */
        if (xRxQueue != NULL) {
            xQueueSendFromISR(xRxQueue, &byte, &xHigherPriorityTaskWoken);
        }
    }

    /* 若解析任务优先级高于当前被中断的任务, 请求上下文切换 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  解析任务: 从队列阻塞读字节, 帧状态机解析
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  帧解析状态机状态
 */
typedef enum {
    PARSE_WAIT_HEAD,   /**< 等待帧头 0xFE */
    PARSE_READ_BODY    /**< 接收帧体（帧头后 7 字节） */
} parse_state_t;

void maixcam_parse_task(void *arg)
{
    (void)arg;

    uint8_t       buf[MAIXCAM_FRAME_LEN];
    uint8_t       idx   = 0;
    parse_state_t state = PARSE_WAIT_HEAD;

    for (;;) {
        uint8_t byte;

        /* 阻塞等待队列中的字节 (portMAX_DELAY: 无数据时让出 CPU) */
        if (xQueueReceive(xRxQueue, &byte, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (state) {
        case PARSE_WAIT_HEAD:
            if (byte == MAIXCAM_FRAME_HEAD) {
                buf[0] = byte;
                idx    = 1;
                state  = PARSE_READ_BODY;
            }
            break;

        case PARSE_READ_BODY:
            buf[idx++] = byte;
            if (idx == MAIXCAM_FRAME_LEN) {
                /* 收满 8 字节, 校验帧尾 */
                if (buf[MAIXCAM_FRAME_LEN - 1] == MAIXCAM_FRAME_TAIL) {
                    /* buf[1..2] 为 x，buf[3..6] 为有符号 vx(px/s)。 */
                    maixcam_ball_t tmp;
                    tmp.x         = be16(&buf[1]);
                    tmp.vx_px_s   = be32s(&buf[3]);
                    tmp.frame_id  = g_ball.frame_id + 1;
                    tmp.tick_ms   = (uint32_t)xTaskGetTickCount() *
                                    portTICK_PERIOD_MS;
                    tmp.valid     = (tmp.x != UINT16_MAX);

                    /* 原子更新全局结构体 (关中断防止读到半更新状态) */
                    taskENTER_CRITICAL();
                    g_ball = tmp;
                    taskEXIT_CRITICAL();
                }
                /* 无论帧尾是否匹配, 都回到等待帧头状态 */
                state = PARSE_WAIT_HEAD;
            }
            break;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  对外接口: 读取最新帧快照
 * ═══════════════════════════════════════════════════════════════════════════ */

bool maixcam_get_ball(maixcam_ball_t *out)
{
    if (out == NULL) {
        return false;
    }

    /* 关中断拷贝快照, 确保不被 ISR/parse_task 部分更新打断 */
    taskENTER_CRITICAL();
    bool v = g_ball.valid;
    *out  = g_ball;
    taskEXIT_CRITICAL();

    return v;
}