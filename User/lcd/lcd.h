/**
 * @file  lcd.h
 * @brief 达妙 DM-MC02 板载 1.69 寸 SPI 屏（ST7789，240x280）驱动。
 *        从达妙例程 CtrBoard-H7_LCD 移植，函数名和参数保持不变，例程里的代码可以直接用。
 *
 * 和例程相比的改动：
 *   - 批量发送：一次设好窗口后用缓冲区连续发送，CS 只拉一次。
 *     例程每个字节都单独调一次 HAL_SPI_Transmit，刷一次全屏要好几秒，现在大约 60ms。
 *   - 汉字改用 UTF-8（例程是 GBK），代码里直接写 "达妙科技" 就能显示（字库里要有这个字）。
 *   - 修了例程 LCD_ShowFloatNum 对负数/大数溢出的问题（int16 -> int32）。
 *   - 新增 LCD_Printf()，用法和 printf 一样。
 *
 * ⚠ 只能在主循环里调用，不要在中断里调用（SPI 是阻塞发送）。
 *
 * 需要 CubeMX：SPI1（PB3 SCK、PD7 MOSI，Transmit Only Master，CPOL High，CPHA 1Edge），
 *             GPIO 输出 PB10=LCD_BLK、PB11=LCD_RES、PD10=LCD_DC、PE15=LCD_CS。
 */
#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define USE_HORIZONTAL 2  /* 0/1 竖屏，2/3 横屏（和例程一样） */

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W 240
#define LCD_H 280
#else
#define LCD_W 280
#define LCD_H 240
#endif

/* 引脚（名字来自 CubeMX 里填的 User Label） */
#define LCD_RES_Clr() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET)
#define LCD_RES_Set() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_CS_Clr()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_BLK_Clr() HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET)
#define LCD_BLK_Set() HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)

void LCD_Init(void);
void LCD_WR_DATA8(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t dat);
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

/** 显示汉字串（UTF-8），sizey = 12/16/24/32；mode 0 = 带背景色，1 = 叠加（只画字） */
void LCD_ShowChinese(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);

/** 显示 ASCII 字符/字符串，sizey = 12/16/24/32 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);

uint32_t mypow(uint8_t m, uint8_t n);
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);

/** 显示图片（RGB565，高字节在前，取模方式和例程一样） */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);

/** 格式化显示（ASCII），例如 LCD_Printf(0, 0, WHITE, BLACK, 16, "rpm=%6.1f", v); */
void LCD_Printf(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));

/* 颜色（RGB565） */
#define WHITE      0xFFFF
#define BLACK      0x0000
#define BLUE       0x001F
#define BRED       0XF81F
#define GRED       0XFFE0
#define GBLUE      0X07FF
#define RED        0xF800
#define MAGENTA    0xF81F
#define GREEN      0x07E0
#define CYAN       0x7FFF
#define YELLOW     0xFFE0
#define BROWN      0XBC40 /* 棕色 */
#define BRRED      0XFC07 /* 棕红色 */
#define GRAY       0X8430 /* 灰色 */
#define DARKBLUE   0X01CF /* 深蓝色 */
#define LIGHTBLUE  0X7D7C /* 浅蓝色 */
#define GRAYBLUE   0X5458 /* 灰蓝色 */
#define LIGHTGREEN 0X841F /* 浅绿色 */
#define LGRAY      0XC618 /* 浅灰色 */
#define LGRAYBLUE  0XA651 /* 浅灰蓝色 */
#define LBBLUE     0X2B12 /* 浅棕蓝色 */

#endif
