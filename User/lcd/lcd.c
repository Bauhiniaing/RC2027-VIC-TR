/**
 * @file  lcd.c
 * @brief ST7789 SPI 屏驱动，见 lcd.h。初始化序列和画图算法来自达妙例程。
 */
#include "lcd.h"
/* 字库是达妙例程的扁平初始化写法，数据正确，只是没写内层大括号 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#include "lcdfont.h"
#pragma GCC diagnostic pop
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SPI_TIMEOUT 100

/* ---------------- 底层：批量发送 ---------------- */

/* 像素缓冲：攒够一批再发，CS 在一次窗口写入期间一直保持低 */
static uint8_t  s_buf[1024];
static uint16_t s_len;

static void spi_send(const uint8_t *d, uint16_t n)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)d, n, SPI_TIMEOUT);
}

static void flush(void)
{
    if (s_len) {
        spi_send(s_buf, s_len);
        s_len = 0;
    }
}

static inline void push_pixel(uint16_t c)
{
    s_buf[s_len++] = (uint8_t)(c >> 8);
    s_buf[s_len++] = (uint8_t)c;
    if (s_len >= sizeof(s_buf)) flush();
}

static void stream_end(void)
{
    flush();
    LCD_CS_Set();
}

void LCD_WR_REG(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();  /* 命令 */
    spi_send(&dat, 1);
    LCD_DC_Set();  /* 后面默认是数据 */
    LCD_CS_Set();
}

void LCD_WR_DATA8(uint8_t dat)
{
    LCD_CS_Clr();
    spi_send(&dat, 1);
    LCD_CS_Set();
}

void LCD_WR_DATA(uint16_t dat)
{
    uint8_t d[2] = { (uint8_t)(dat >> 8), (uint8_t)dat };
    LCD_CS_Clr();
    spi_send(d, 2);
    LCD_CS_Set();
}

/* 设窗口。这块屏 240x280，控制器是 240x320，所以 280 方向有 20 像素偏移（例程里的 +20） */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
    y1 += 20; y2 += 20;
#else
    x1 += 20; x2 += 20;
#endif
    LCD_WR_REG(0x2a);  /* 列地址 */
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);
    LCD_WR_REG(0x2b);  /* 行地址 */
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);
    LCD_WR_REG(0x2c);  /* 写显存 */
}

/* 设窗口并开始连续写像素，写完调 stream_end() */
static void stream_begin(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Address_Set(x1, y1, x2, y2);
    s_len = 0;
    LCD_CS_Clr();
}

/* ---------------- 基本图形 ---------------- */

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    if (xend > LCD_W) xend = LCD_W;
    if (yend > LCD_H) yend = LCD_H;
    if (xsta >= xend || ysta >= yend) return;
    uint32_t n = (uint32_t)(xend - xsta) * (yend - ysta);
    stream_begin(xsta, ysta, xend - 1, yend - 1);
    while (n--) push_pixel(color);
    stream_end();
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_W || y >= LCD_H) return;
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    /* 横线/竖线直接填充，快很多 */
    if (y1 == y2) { if (x1 > x2) { uint16_t t = x1; x1 = x2; x2 = t; } LCD_Fill(x1, y1, x2 + 1, y1 + 1, color); return; }
    if (x1 == x2) { if (y1 > y2) { uint16_t t = y1; y1 = y2; y2 = t; } LCD_Fill(x1, y1, x1 + 1, y2 + 1, color); return; }

    int xerr = 0, yerr = 0, distance;
    int delta_x = x2 - x1, delta_y = y2 - y1;
    int incx = delta_x > 0 ? 1 : -1, incy = delta_y > 0 ? 1 : -1;
    int uRow = x1, uCol = y1;
    if (delta_x < 0) delta_x = -delta_x;
    if (delta_y < 0) delta_y = -delta_y;
    distance = delta_x > delta_y ? delta_x : delta_y;
    for (int t = 0; t <= distance; t++) {
        LCD_DrawPoint(uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) { xerr -= distance; uRow += incx; }
        if (yerr > distance) { yerr -= distance; uCol += incy; }
    }
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0, b = r;
    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;
        if ((a * a + b * b) > (r * r)) b--;
    }
}

/* ---------------- 点阵绘制（字符、汉字共用） ---------------- */

/* 点阵格式（和例程取模方式一致）：逐行，每行 ceil(w/8) 字节，低位在前 */
static void draw_glyph(uint16_t x, uint16_t y, uint8_t w, uint8_t h, const uint8_t *msk,
                       uint16_t fc, uint16_t bc, uint8_t mode)
{
    uint8_t bpr = (w + 7) / 8;  /* 每行字节数 */
    if (!mode) {  /* 带背景：整块一次发出 */
        if (x + w > LCD_W || y + h > LCD_H) return;
        stream_begin(x, y, x + w - 1, y + h - 1);
        for (uint8_t r = 0; r < h; r++) {
            for (uint8_t c = 0; c < w; c++) {
                push_pixel((msk[r * bpr + c / 8] >> (c % 8)) & 1 ? fc : bc);
            }
        }
        stream_end();
    } else {      /* 叠加：只画前景点 */
        for (uint8_t r = 0; r < h; r++) {
            for (uint8_t c = 0; c < w; c++) {
                if ((msk[r * bpr + c / 8] >> (c % 8)) & 1) LCD_DrawPoint(x + c, y + r, fc);
            }
        }
    }
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    if (num < ' ' || num > '~') num = '?';
    num -= ' ';
    const uint8_t *msk;
    switch (sizey) {
    case 12: msk = ascii_1206[num]; break;
    case 16: msk = ascii_1608[num]; break;
    case 24: msk = ascii_2412[num]; break;
    case 32: msk = ascii_3216[num]; break;
    default: return;
    }
    draw_glyph(x, y, sizey / 2, sizey, msk, fc, bc, mode);
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*p != '\0') {
        if (x + sizey / 2 > LCD_W) break;  /* 超出屏幕右边就不画了 */
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/* UTF-8 一个字符占几个字节 */
static int utf8_len(uint8_t c)
{
    if (c < 0x80) return 1;
    if ((c & 0xE0) == 0xC0) return 2;
    if ((c & 0xF0) == 0xE0) return 3;
    if ((c & 0xF8) == 0xF0) return 4;
    return 1;
}

/* 在字库里找汉字，找到返回点阵，否则 NULL */
#define FIND_HZ(tab, s, n)                                                        \
    do {                                                                          \
        for (unsigned k = 0; k < sizeof(tab) / sizeof(tab[0]); k++) {             \
            if (strncmp(tab[k].Index, s, n) == 0 && tab[k].Index[n] == '\0')      \
                return (const uint8_t *)tab[k].Msk;                               \
        }                                                                         \
    } while (0)

static const uint8_t *find_hz(const char *s, int n, uint8_t sizey)
{
    switch (sizey) {
    case 12: FIND_HZ(tfont12, s, n); break;
    case 16: FIND_HZ(tfont16, s, n); break;
    case 24: FIND_HZ(tfont24, s, n); break;
    case 32: FIND_HZ(tfont32, s, n); break;
    }
    return NULL;
}

void LCD_ShowChinese(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    if (sizey != 12 && sizey != 16 && sizey != 24 && sizey != 32) return;
    while (*s) {
        int n = utf8_len((uint8_t)*s);
        if (n == 1) {  /* 中英文混排：ASCII 用半角字体 */
            LCD_ShowChar(x, y, (uint8_t)*s, fc, bc, sizey, mode);
            x += sizey / 2;
        } else {
            const uint8_t *msk = find_hz(s, n, sizey);
            if (msk) draw_glyph(x, y, sizey, sizey, msk, fc, bc, mode);
            else if (!mode) LCD_Fill(x, y, x + sizey, y + sizey, bc);  /* 字库里没有：留空 */
            x += sizey;
        }
        s += n;
    }
}

/* ---------------- 数字 ---------------- */

uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t enshow = 0, sizex = sizey / 2;
    for (uint8_t t = 0; t < len; t++) {
        uint8_t temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            }
            enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + '0', fc, bc, sizey, 0);
    }
}

/* 带符号浮点数：len 为整数位数，decimal 为小数位数（例程用 int16，超过 ±32767 会溢出，这里改成 printf） */
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%+0*.*f", len + decimal + 2, decimal, (double)num);
    if (buf[0] == '+') buf[0] = ' ';
    LCD_ShowString(x, y, (const uint8_t *)buf, fc, bc, sizey, 0);
}

/* 非负浮点数 */
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    char buf[24];
    if (num < 0) num = 0;
    snprintf(buf, sizeof(buf), "%0*.*f", len + decimal + 1, decimal, (double)num);
    LCD_ShowString(x, y, (const uint8_t *)buf, fc, bc, sizey, 0);
}

void LCD_Printf(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    LCD_ShowString(x, y, (const uint8_t *)buf, fc, bc, sizey, 0);
}

void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    if (x + length > LCD_W || y + width > LCD_H) return;
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    LCD_CS_Clr();
    uint32_t total = (uint32_t)length * width * 2;
    while (total) {  /* 图片已经是高字节在前的 RGB565，直接分块发 */
        uint16_t n = total > 0xFFFF ? 0xFFFF : (uint16_t)total;
        spi_send(pic, n);
        pic += n;
        total -= n;
    }
    LCD_CS_Set();
}

/* ---------------- 初始化（序列来自达妙例程） ---------------- */

static void wr_cmd_data(uint8_t cmd, const uint8_t *d, uint8_t n)
{
    LCD_WR_REG(cmd);
    for (uint8_t i = 0; i < n; i++) LCD_WR_DATA8(d[i]);
}

void LCD_Init(void)
{
    /* H7 的 HAL 每次发送完都会关闭 SPI。KeepIOState 关闭时 SCK 会变成浮空，下次打开 SPI 会多出一个时钟沿。
     * 例程每个字节都拉一次 CS，所以没事；这里 CS 在整段数据期间一直为低，多出来的沿会让数据错位、整屏发黑。
     * 所以强制打开 KeepIOState（CubeMX 里 SPI1 -> Master Keep Io State = Enabled 也可以）。 */
    if (hspi1.Init.MasterKeepIOState != SPI_MASTER_KEEP_IO_STATE_ENABLE) {
        HAL_SPI_DeInit(&hspi1);
        hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
        if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
    }

    LCD_CS_Set();
    LCD_DC_Set();

    LCD_RES_Clr();  /* 复位 */
    HAL_Delay(100);
    LCD_RES_Set();
    HAL_Delay(100);

    LCD_BLK_Set();  /* 背光 */
    HAL_Delay(100);

    LCD_WR_REG(0x11);  /* Sleep out */
    HAL_Delay(120);

    static const uint8_t k_madctl[4] = { 0x00, 0xC0, 0x70, 0xA0 };
    wr_cmd_data(0x36, &k_madctl[USE_HORIZONTAL], 1);  /* 显示方向 */

    static const uint8_t k_3a[] = { 0x05 };  /* RGB565 */
    static const uint8_t k_b2[] = { 0x0C, 0x0C, 0x00, 0x33, 0x33 };
    static const uint8_t k_b7[] = { 0x35 };
    static const uint8_t k_bb[] = { 0x32 };  /* Vcom=1.35V */
    static const uint8_t k_c2[] = { 0x01 };
    static const uint8_t k_c3[] = { 0x15 };  /* GVDD=4.8V */
    static const uint8_t k_c4[] = { 0x20 };  /* VDV */
    static const uint8_t k_c6[] = { 0x0F };  /* 60Hz */
    static const uint8_t k_d0[] = { 0xA4, 0xA1 };
    static const uint8_t k_e0[] = { 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34 };
    static const uint8_t k_e1[] = { 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34 };
    wr_cmd_data(0x3A, k_3a, sizeof(k_3a));
    wr_cmd_data(0xB2, k_b2, sizeof(k_b2));
    wr_cmd_data(0xB7, k_b7, sizeof(k_b7));
    wr_cmd_data(0xBB, k_bb, sizeof(k_bb));
    wr_cmd_data(0xC2, k_c2, sizeof(k_c2));
    wr_cmd_data(0xC3, k_c3, sizeof(k_c3));
    wr_cmd_data(0xC4, k_c4, sizeof(k_c4));
    wr_cmd_data(0xC6, k_c6, sizeof(k_c6));
    wr_cmd_data(0xD0, k_d0, sizeof(k_d0));
    wr_cmd_data(0xE0, k_e0, sizeof(k_e0));
    wr_cmd_data(0xE1, k_e1, sizeof(k_e1));
    LCD_WR_REG(0x21);  /* 反色（这块屏需要） */
    LCD_WR_REG(0x29);  /* 开显示 */
}
