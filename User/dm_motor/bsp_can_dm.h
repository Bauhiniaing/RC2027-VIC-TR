#ifndef BSP_CAN_DM_H
#define BSP_CAN_DM_H
#include "stm32h7xx_hal.h"
#include "bsp_fdcan.h"

/* 把达妙电机接收回调挂到 FDCAN1 上, 在 bsp_fdcan_init() 之后调用一次 */
extern void FDCAN1_Filter_Init(void);
#endif
