#include "bsp_can_dm.h"
#include "J4340.h"

/*
 * F4(bxCAN) -> H7(FDCAN) 移植说明:
 *  - 原来这里自己配过滤器 + HAL_CAN_Start + 开 FIFO1 中断。
 *  - H7 上 FDCAN1/2/3 已由 bsp_fdcan_init()(app_init 里调用)统一启动:
 *    全局过滤器把所有帧收进 RX FIFO0, 在 HAL_FDCAN_RxFifo0Callback 里按回调分发。
 *  - 所以这里不能再 Start 一次(重复 Start 会返回 HAL_ERROR), 只需把达妙的
 *    接收回调挂到 FDCAN1 上。必须在 bsp_fdcan_init() 之后调用。
 */

/* 达妙反馈帧: 标准帧, 8 字节, ID = MST_ID(默认 DM4340_FEEDBACK_BASE)。
   DJI 的反馈 ID 从 0x201 开始, 这里只认 < 0x200 的帧, 避免把 DJI 帧当达妙解析。 */
static uint8_t dm_fdcan1_rx_cb(FDCAN_HandleTypeDef *hfdcan, const can_frame_t *frame)
{
    (void)hfdcan;
    if (frame->is_ext || frame->len != 8U || frame->id >= 0x200U)
    {
        return 0;
    }
    DM4340_ParseFeedback((uint16_t)frame->id, (uint8_t *)frame->data);
    return 1;
}

void FDCAN1_Filter_Init(void)
{
    bsp_fdcan_register_rx(&hfdcan1, dm_fdcan1_rx_cb);
}
