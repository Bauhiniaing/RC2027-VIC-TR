/**
 * @file  vofa.h
 * @brief VOFA+ 调参：USB 虚拟串口发波形（JustFloat 协议），收命令在线改 PID/目标值。
 *
 * 波形通道（都是输出轴单位）：
 *   I0 位置目标 °    I1 位置反馈 °
 *   I2 速度目标 rpm  I3 速度反馈 rpm
 *   I4 PID 输出（给电调的电流/电压）  I5 电调回传的实际电流
 *
 * 命令（文本，以 \n 或 ; 结尾，例如 "pkp=0.3\n"）：
 *   m=<下标>   选择电机（motor_cfg.h 枚举的数字，2006_1 是 8）
 *   pos=<°>    位置模式目标     spd=<rpm>  速度模式目标     off  关闭电机
 *   lim=<rpm>  位置模式限速（0 = 用默认）
 *   skp= ski= skd=   速度环参数（转子单位，和 F4 一样）
 *   pkp= pki= pkd=   位置环参数
 *   div=<n>    每 n ms 采一个点（默认 1，即 1kHz）
 */
#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

void vofa_init(int motor_idx);
void vofa_sample_isr(void);                               /* 1ms 中断里调用（PID 算完之后） */
void vofa_poll(void);                                     /* 主循环里调用：发波形、处理命令 */
void vofa_on_usb_rx(const uint8_t *buf, uint32_t len);    /* USB 接收回调里调用 */

#endif
