#ifndef OLED_H
#define OLED_H

/**
 * @file    oled.h
 * @brief   SSD1306 OLED 128×64 软件 I2C 显示屏驱动接口 (MSPM0G3507)
 *          软件 I2C: PA28=SDA, PA31=SCL
 */

#include <stdint.h>

/* ── SSD1306 I2C 配置 ── */
#define OLED_ADDR      0x3C          /* SSD1306 7 位 I2C 地址 (SA0=0) */
#define OLED_WIDTH     128U          /* 显示屏宽度（像素） */
#define OLED_HEIGHT     64U          /* 显示屏高度（像素） */

/* 启用中文字库渲染（需 oledfont.h 中 Hzk/CN16 字模数据） */
#define OLED_ENABLE_CHINESE 0

#define OLED_CMD   0                 /* 命令模式（控制字节 = 0x00） */
#define OLED_DATA  1                 /* 数据模式（控制字节 = 0x40） */

void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size1, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode);
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode);
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode);
void OLED_Init(void);
void oled_i2c_sda_unlock(void);  /* 释放被从机拉低的 SDA 总线（总线死锁恢复） */
void oled_ShowString_msk16(uint8_t x, uint8_t y, char *s, uint8_t mode);
int OLED_vsprint(uint8_t x, uint8_t y, uint8_t size, const char *formate, ...);
int OLED__msk_vsprint(uint8_t row, uint8_t col, const char *formate, ...);

#endif