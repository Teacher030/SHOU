/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.96寸 7针 SPI OLED (SSD1306) 驱动头文件
  * @note    使用 SPI2:
  *            D0(SCK) -> PB13 (SPI2_SCK)
  *            D1(SDA) -> PB15 (SPI2_MOSI)
  *            RES     -> PB14 (普通 GPIO 输出)
  *            CS      -> PB12 (普通 GPIO 输出)
  *            DC      -> PB3  (普通 GPIO 输出)
  *          分辨率 128x64, 采用内部 1KB 帧缓冲 + 整屏刷新
  ******************************************************************************
  */
#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 屏幕尺寸 -------------------------------------------------------------------*/
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)   /* 8 个显示页 */

/* 接口函数 -------------------------------------------------------------------*/
void OLED_Init(void);                                    /* 初始化并点亮屏幕 */
void OLED_Clear(void);                                   /* 清空帧缓冲(需 Refresh 才生效) */
void OLED_Refresh(void);                                 /* 将帧缓冲整屏刷新到 OLED */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);/* 画点 color:1 亮 0 灭 */
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color);   /* 6x8 字符 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t color);/* 字符串 */
void OLED_ShowInt(uint8_t x, uint8_t y, int32_t num, uint8_t width, uint8_t color); /* 带符号整数 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float val, uint8_t color);   /* 带符号浮点(1位小数) */

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
