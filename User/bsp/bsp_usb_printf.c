/**
 * @file  bsp_usb_printf.c
 * @brief printf 重定向到 USB 虚拟串口（GCC 写法：覆盖 syscalls.c 里的弱函数 _write）。
 *
 * 注意：
 *  - 只在主循环里用 printf，不要在中断里用。
 *  - USB 没连上/串口助手没打开时，数据直接丢弃，不会卡住程序。
 *  - 打印浮点 %f 需要链接选项 -u _printf_float（已在 CMakeLists.txt 里加了）。
 */
#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define USB_TX_TIMEOUT_MS 5

static uint8_t s_tx_buf[512];

static int usb_ready(void)
{
    return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceHS.pClassData != NULL;
}

/* 等上一包发完（TxState 清零），超时返回 0 */
static int wait_tx_idle(void)
{
    uint32_t t0 = HAL_GetTick();
    while (((USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData)->TxState != 0) {
        if (HAL_GetTick() - t0 > USB_TX_TIMEOUT_MS || !usb_ready()) return 0;
    }
    return 1;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    if (!usb_ready()) return len;  /* 没连上：丢弃 */

    int sent = 0;
    while (sent < len) {
        if (!wait_tx_idle()) break;
        int n = len - sent;
        if (n > (int)sizeof(s_tx_buf)) n = sizeof(s_tx_buf);
        memcpy(s_tx_buf, ptr + sent, n);  /* 复制一份，USB 是异步发送，原缓冲区可能被改 */
        if (CDC_Transmit_HS(s_tx_buf, (uint16_t)n) != USBD_OK) break;
        sent += n;
    }
    return len;
}
