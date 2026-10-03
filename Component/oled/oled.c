/**
 * @file    oled.c
 * @brief   SSD1306 OLED 128×64 软件 I2C 显示屏驱动 (MSPM0G3507 移植版)
 * @note
 *   ── 显存结构 ──
 *   OLED_GRAM[128][8]（横向 128 列 × 纵向 8 页，每页 8 行）
 *   所有绘图函数操作显存，调用 OLED_Refresh() 将显存刷新到屏幕
 *
 *   ── 软件 I2C ──
 *   引脚: PA28=SDA, PA31=SCL
 *   OLED 使用 GPIO bit-bang 模拟 I2C，避免占用硬件外设。
 *   时钟频率: ~125kHz (delay_cycles(120) @ 80MHz)
 *
 *   ── SSD1306 数据格式 ──
 *   I2C 一次传输: [控制字节] [数据字节...]
 *     控制字节 = 0x00 → 后续是命令 (CMD)
 *     控制字节 = 0x40 → 后续是显示数据 (DATA)
 *
 *   ── 刷新性能 ──
 *   全屏刷新: 8 页 × (1 控制字节 + 128 数据字节) = 1032 字节
 *   @ 125kHz I2C → 约 66ms (含 Start/Stop 开销约 30ms, 合计约 96ms)
 *   建议刷新周期: ≥ 100ms (本工程: 100ms, OLED 任务)
 *
 *   ── 功能清单 ──
 *   - 基本绘图: 点、线、圆
 *   - 字符显示: ASCII (8/12/16/24/32 号字体)
 *   - 中文显示: 需启用 OLED_ENABLE_CHINESE (16/24/32/64 号字体)
 *   - 图片显示: BMP 格式位图
 *   - 格式化输出: OLED_vsprint (类似 printf)
 *   - 滚屏显示: 中文字符滚屏
 *
 *   ── Bus Pirate 调试技巧 ──
 *   如果 OLED 初始化后不显示:
 *   1) 检查 I2C 地址: 扫描 0x3C/0x3D (SA0 引脚决定)
 *   2) 检查 SDA/SCL 是否被拉低: 用万用表测 GPIO 电平
 *   3) 上电后 SDA 低电平 → 死锁 → 调用 oled_i2c_sda_unlock() 恢复
 */

#include "oled.h"
#include "oledfont.h"
#include "ti_msp_dl_config.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

uint8_t OLED_GRAM[OLED_WIDTH][OLED_HEIGHT / 8U];  /* OLED 显存：128×8 字节 */

/* OLED 使用 SysConfig 生成的 GPIO_OLED_SW_I2C 引脚做软件 I2C。 */
#define OLED_SW_I2C_PORT       GPIO_OLED_SW_I2C_PORT
#define OLED_SW_SDA_PIN        GPIO_OLED_SW_I2C_PIN_OLED_SDA_PIN
#define OLED_SW_SCL_PIN        GPIO_OLED_SW_I2C_PIN_OLED_SCL_PIN
#define OLED_SW_SDA_IOMUX      GPIO_OLED_SW_I2C_PIN_OLED_SDA_IOMUX
#define OLED_SW_SCL_IOMUX      GPIO_OLED_SW_I2C_PIN_OLED_SCL_IOMUX
#define OLED_SW_I2C_DELAY      (120U)

static void oled_i2c_delay(void)
{
    delay_cycles(OLED_SW_I2C_DELAY);
}

static void oled_sda_release(void)
{
    DL_GPIO_disableOutput(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN);
}

static void oled_sda_low(void)
{
    DL_GPIO_clearPins(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN);
    DL_GPIO_enableOutput(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN);
}

static void oled_scl_release(void)
{
    DL_GPIO_disableOutput(OLED_SW_I2C_PORT, OLED_SW_SCL_PIN);
}

static void oled_scl_low(void)
{
    DL_GPIO_clearPins(OLED_SW_I2C_PORT, OLED_SW_SCL_PIN);
    DL_GPIO_enableOutput(OLED_SW_I2C_PORT, OLED_SW_SCL_PIN);
}

static uint8_t oled_sda_read(void)
{
    return (DL_GPIO_readPins(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN) != 0U) ? 1U : 0U;
}

static void oled_i2c_gpio_init(void)
{
    DL_GPIO_initDigitalOutput(OLED_SW_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SW_SCL_IOMUX);
    DL_GPIO_clearPins(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN | OLED_SW_SCL_PIN);
    DL_GPIO_disableOutput(OLED_SW_I2C_PORT, OLED_SW_SDA_PIN | OLED_SW_SCL_PIN);
}

/**
 * @brief  OLED I2C 总线死锁恢复
 * @note   当 SSD1306 异常复位时 SDA 可能被拉低 → 总线死锁
 *         通过 GPIO bit-bang 产生 SCL 脉冲 (最多 100 个),
 *         每次 SCL 高电平时检查 SDA, 若释放则恢复成功
 *         原理: 从机在 SCL 上升沿输出 1 bit, 连续输出直到释放 SDA
 */
void oled_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0U;

    oled_i2c_gpio_init();
    do {
        oled_scl_low();
        delay_cycles(8000);
        oled_scl_release();
        delay_cycles(8000);
        if (oled_sda_read()) {
            break;
        }
    } while (++cycleCnt < 100U);
}

/**
 * @brief  I2C START 条件: SDA 高→低 时 SCL 保持高
 * @note   总线空闲时 SDA/SCL 均为高 (外部上拉)
 *         START: SCL=高, SDA=高→低
 */
static void oled_i2c_start(void)
{
    oled_sda_release();  /* SDA=高 (释放) */
    oled_scl_release();  /* SCL=高 (释放) */
    oled_i2c_delay();
    oled_sda_low();      /* SDA=低 → START 条件 */
    oled_i2c_delay();
    oled_scl_low();      /* SCL=低 → 准备发送数据 */
    oled_i2c_delay();
}

/**
 * @brief  I2C STOP 条件: SDA 低→高 时 SCL 保持高
 * @note   STOP: SCL=高, SDA=低→高
 *         之后总线空闲 (SDA/SCL 均为高, 外部上拉)
 */
static void oled_i2c_stop(void)
{
    oled_sda_low();      /* SDA=低 (准备) */
    oled_i2c_delay();
    oled_scl_release();  /* SCL=高 */
    oled_i2c_delay();
    oled_sda_release();  /* SDA=高 → STOP 条件 */
    oled_i2c_delay();    /* 总线空闲 */
}

/**
 * @brief  软件 I2C 写一个字节 (MSB first) + 接收 ACK
 * @param  data  待发送的 8 位数据
 * @note   I2C 时序 (每 bit):
 *         1) SCL=低, 设置 SDA (data bit 7→0)
 *         2) SCL=高, 从机采样 SDA
 *         3) SCL=低, 准备下一位
 *
 *         第 9 个时钟: 释放 SDA → 从机拉低 SDA (ACK) 或 释放 (NACK)
 *         本实现不检查 ACK, 避免 OLED 异常时阻塞显示任务
 */
static void oled_i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8U; i++) {
        /* MSB first: 先发最高位 */
        if (data & 0x80U) {
            oled_sda_release();  /* 数据位=1 → 释放 SDA (外部上拉→高) */
        } else {
            oled_sda_low();      /* 数据位=0 → 拉低 SDA */
        }
        oled_i2c_delay();

        /* SCL 上升沿: 从机锁存数据 */
        oled_scl_release();
        oled_i2c_delay();

        /* SCL 下降沿: 准备下一个 bit */
        oled_scl_low();
        data <<= 1;  /* 左移, 下次发送 bit 6 */
        oled_i2c_delay();
    }

    /* ── 第 9 个时钟: ACK 时隙 ──
     * 主机释放 SDA → 从机拉低表示 ACK, 保持高表示 NACK
     * 不强制检查 ACK: 若从机不应答, 跳过本次而不阻塞任务 */
    oled_sda_release();
    oled_i2c_delay();
    oled_scl_release();  /* SCL 上升沿: 从机输出 ACK */
    oled_i2c_delay();
    oled_scl_low();      /* SCL 下降沿: 结束 ACK 时隙 */
    oled_i2c_delay();
}

/**
 * @brief  软件 I2C 完整传输: START | addr(W) | data[] | STOP
 * @param  buf  数据缓冲区 (不含从机地址)
 * @param  len  数据长度
 * @note   地址左移 1 位 (7bit addr → 8bit addr+R/W=0)
 *         例: OLED_ADDR=0x3C → 发送 0x78
 */
static void oled_i2c_transmit(const uint8_t *buf, uint16_t len)
{
    oled_i2c_start();
    /* 发送从机地址 + 写位 (R/W=0) */
    oled_i2c_write_byte((uint8_t)(OLED_ADDR << 1));
    /* 发送数据字节 */
    for (uint16_t i = 0; i < len; i++) {
        oled_i2c_write_byte(buf[i]);
    }
    oled_i2c_stop();
}

/**
 * @brief  设置 OLED 显示颜色模式（正常/反色）
 * @param  i  0=正常显示（白底黑字），1=反色显示
 */
void OLED_ColorTurn(uint8_t i)
{
    if (i == 0)
        OLED_WR_Byte(0xA6, OLED_CMD);
    if (i == 1)
        OLED_WR_Byte(0xA7, OLED_CMD);
}

/**
 * @brief  旋转 OLED 显示方向 180°
 * @param  i  0=正常方向，1=旋转 180°
 */
void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

/**
 * @brief  通过 I2C 向 SSD1306 发送一个字节
 * @param  dat   数据 / 命令字节
 * @param  mode  OLED_CMD=命令, OLED_DATA=数据
 * @note   SSD1306 I2C 协议：第 1 字节为控制字节（0x00=命令, 0x40=数据），第 2 字节为有效数据
 */
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (mode == OLED_CMD) ? 0x00 : 0x40;
    txBuffer[1] = dat;
    oled_i2c_transmit(txBuffer, 2);
}

/**
 * @brief  开启 OLED 显示
 */
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

/**
 * @brief  关闭 OLED 显示（进入睡眠模式）
 */
void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

/**
 * @brief  将显存 GRAM 整体刷新到 OLED 屏幕（全屏更新）
 * @note   每页先发命令设置页/列地址，再单次 I2C 传输 129 字节（1 控制字节 + 128 数据字节）
 */
void OLED_Refresh(void)
{
    uint8_t i;
    uint8_t txBuffer[OLED_WIDTH + 1U];

    for (i = 0; i < (OLED_HEIGHT / 8U); i++) {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);   /* 设置当前页地址 */
        OLED_WR_Byte(0x00, OLED_CMD);        /* 列地址低 4 位 */
        OLED_WR_Byte(0x10, OLED_CMD);        /* 列地址高 4 位 */

        txBuffer[0] = 0x40;                  /* 数据模式控制字节 */
        for (int col = 0; col < OLED_WIDTH; col++) {
            txBuffer[col + 1] = OLED_GRAM[col][i];
        }
        oled_i2c_transmit(txBuffer, OLED_WIDTH + 1U);
    }
}

/**
 * @brief  清空显存并刷新 OLED（全屏黑色）
 */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < (OLED_HEIGHT / 8U); i++) {
        for (n = 0; n < OLED_WIDTH; n++) {
            OLED_GRAM[n][i] = 0;
        }
    }
    OLED_Refresh();
}

/**
 * @brief  在显存中绘制一个像素点
 * @param  x  横坐标（0~127）
 * @param  y  纵坐标（0~63）
 * @param  t  0=擦除（黑），1=点亮（白）
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }

    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t)
        OLED_GRAM[x][i] |= n;
    else {
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
        OLED_GRAM[x][i] |= n;
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
}

/**
 * @brief  绘制直线（Bresenham 算法）
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }
    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }
    if (delta_x > delta_y) distance = delta_x;
    else distance = delta_y;
    for (t = 0; t < distance + 1; t++) {
        OLED_DrawPoint(uRow, uCol, mode);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/**
 * @brief  绘制圆（Bresenham 画圆算法）
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);
        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);
        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) {
            b--;
            a--;
        }
    }
}

/**
 * @brief  在指定位置显示一个 ASCII 字符
 * @param  size1 字号：8/12/16/24/32
 * @param  mode  0=白底黑字，1=黑底白字
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if (size1 == 8) size2 = 6;
    else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++) {
        if (size1 == 8)
            temp = asc2_0806[chr1][i];
        else if (size1 == 12)
            temp = asc2_1206[chr1][i];
        else if (size1 == 16)
            temp = asc2_1608[chr1][i];
        else if (size1 == 24)
            temp = asc2_2412[chr1][i];
        else if (size1 == 32)
            temp = asc2_1632[chr1][i];
        else return;
        for (m = 0; m < 8; m++) {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((size1 != 8) && ((x - x0) == size1 / 2)) {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

/**
 * @brief  显示 ASCII 字符串（自动换行）
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size1, uint8_t mode)
{
    if (chr == NULL || size1 == 0U || y >= OLED_HEIGHT) {
        return;
    }

    uint8_t char_w = (size1 == 8U) ? 6U : (size1 / 2U);
    while ((*chr >= ' ') && (*chr <= '~')) {
        if (x + char_w > OLED_WIDTH) {
            break;
        }
        OLED_ShowChar(x, y, *chr, size1, mode);
        if (size1 == 8) x += 6;
        else x += size1 / 2;
        chr++;
    }
}

/**
 * @brief  幂运算 m^n（用于数字显示）
 */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief  显示数字（右对齐，高位补零）
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
    if (size1 == 8) m = 2;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0)
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        else
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
    }
}

/**
 * @brief  显示汉字（需启用 OLED_ENABLE_CHINESE）
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
#if OLED_ENABLE_CHINESE
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;
    for (i = 0; i < size3; i++) {
        if (size1 == 16)
            temp = Hzk1[num][i];
        else if (size1 == 24)
            temp = Hzk2[num][i];
        else if (size1 == 32)
            temp = Hzk3[num][i];
        else if (size1 == 64)
            temp = Hzk4[num][i];
        else return;
        for (m = 0; m < 8; m++) {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == size1) {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
#else
    (void)x; (void)y; (void)num; (void)size1; (void)mode;
#endif
}

/**
 * @brief  汉字滚屏显示（需启用 OLED_ENABLE_CHINESE）
 */
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
#if OLED_ENABLE_CHINESE
    uint8_t i, n, t = 0, m = 0, r;
    while (1) {
        if (m == 0) {
            OLED_ShowChinese(OLED_WIDTH, 24, t, 16, mode);
            t++;
        }
        if (t == num) {
            for (r = 0; r < 16 * space; r++) {
                for (i = 1; i < OLED_WIDTH; i++) {
                    for (n = 0; n < 8; n++) {
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                    }
                }
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16) m = 0;
        for (i = 1; i < OLED_WIDTH; i++) {
            for (n = 0; n < 8; n++) {
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
            }
        }
        OLED_Refresh();
    }
#else
    (void)num; (void)space; (void)mode;
#endif
}

/**
 * @brief  显示图片
 */
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (n = 0; n < sizey; n++) {
        for (i = 0; i < sizex; i++) {
            temp = BMP[j];
            j++;
            for (m = 0; m < 8; m++) {
                if (temp & 0x01)
                    OLED_DrawPoint(x, y, mode);
                else
                    OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == sizex) {
                x = x0;
                y0 = y0 + 8;
            }
            y = y0;
        }
    }
}

/**
 * @brief  SSD1306 OLED 初始化（PA28/PA31 软件 I2C，此处发命令序列）
 */
void OLED_Init(void)
{
    oled_i2c_gpio_init();

    /* SSD1306 复位时可能拉低 SDA 导致总线死锁，先检测并软件释放 */
    if (!oled_sda_read())
        oled_i2c_sda_unlock();

    /* 复位后等待稳定 */
    delay_cycles(CPUCLK_FREQ / 2); /* 500ms @ 80MHz */

    OLED_WR_Byte(0xAE, OLED_CMD); // display off
    OLED_WR_Byte(0x00, OLED_CMD); // set low column address
    OLED_WR_Byte(0x10, OLED_CMD); // set high column address
    OLED_WR_Byte(0x40, OLED_CMD); // set start line address
    OLED_WR_Byte(0xB0, OLED_CMD); // set page address
    OLED_WR_Byte(0x81, OLED_CMD); // contract control
    OLED_WR_Byte(0xFF, OLED_CMD); // 128
    OLED_WR_Byte(0xA1, OLED_CMD); // set segment remap
    OLED_WR_Byte(0xA6, OLED_CMD); // normal / reverse
    OLED_WR_Byte(0xA8, OLED_CMD); // set multiplex ratio
    OLED_WR_Byte(0x3F, OLED_CMD); // 1/32 duty
    OLED_WR_Byte(0xC8, OLED_CMD); // Com scan direction
    OLED_WR_Byte(0xD3, OLED_CMD); // set display offset
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD); // set osc division
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD8, OLED_CMD); // set area color mode off
    OLED_WR_Byte(0x05, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); // Set Pre-Charge Period
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD); // set com pin configuartion
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); // set Vcomh
    OLED_WR_Byte(0x30, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD); // set charge pump enable
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); // turn on oled panel
}

/**
 * @brief  混合显示 ASCII + 16×16 汉字字符串（需启用 OLED_ENABLE_CHINESE）
 */
void oled_ShowString_msk16(uint8_t x, uint8_t y, char *s, uint8_t mode)
{
#if OLED_ENABLE_CHINESE
    unsigned char i, k, length;
    unsigned short Index = 0;
    uint8_t m, temp, size1 = 16;
    uint8_t x0 = x, y0 = y, y_const = y, x_const = x;
    uint16_t j, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;

    length = strlen(s);
    for (k = 0; k < length; k++) {
        if (*(s + k) <= 127) {
            y = y_const;
            OLED_ShowChar(x, y, *(s + k), size1, 1);
            x += size1 / 2;
        } else if (*(s + k) > 127) {
            Index = (*(s + k) << 8) | (*(s + k + 1));
            for (i = 0; i < sizeof(CN16_Msk) / 34; i++) {
                if (Index == CN16_Msk[i].Index) {
                    x = x0 = x_const;
                    y = y0 = y_const;
                    for (j = 0; j < size3; j++) {
                        temp = CN16_Msk[i].Msk[j];
                        for (m = 0; m < 8; m++) {
                            if (temp & 0x01)
                                OLED_DrawPoint(x, y, mode);
                            else
                                OLED_DrawPoint(x, y, !mode);
                            temp >>= 1;
                            y++;
                        }
                        x++;
                        if ((x - x0) == size1) {
                            x = x0;
                            y0 = y0 + 8;
                        }
                        y = y0;
                    }
                    x = x_const += 16;
                    k += 1;
                }
            }
        }
    }
    OLED_Refresh();
#else
    (void)x; (void)y; (void)s; (void)mode;
#endif
}

int OLED__msk_vsprint(uint8_t row, uint8_t col, const char *formate, ...)
{
#if OLED_ENABLE_CHINESE
    char buf[32] = {'\0'};
    int ret = 0;
    va_list ap;

    va_start(ap, formate);
    ret = vsnprintf(buf, sizeof(buf), formate, ap);
    va_end(ap);
    oled_ShowString_msk16(row, col, buf, 1);
    OLED_Refresh();
    return ret;
#else
    (void)row; (void)col; (void)formate;
    return 0;
#endif
}

int OLED_vsprint(uint8_t x, uint8_t y, uint8_t size, const char *formate, ...)
{
    char buf[32] = {'\0'};
    int ret = 0;
    va_list ap;

    va_start(ap, formate);
    ret = vsnprintf(buf, sizeof(buf), formate, ap);
    va_end(ap);

    OLED_ShowString(x, y, buf, size, 1);
    return ret;
}