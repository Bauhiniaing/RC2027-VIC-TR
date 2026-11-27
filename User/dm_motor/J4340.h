#ifndef __J4340_H
#define __J4340_H

#include "main.h"
#include "bsp_fdcan.h"
#include "bsp_can_dm.h"
/* =====================================================================
 *                 DM-J4340 达妙电机驱动 (FDCAN1, MIT 帧)
 * =====================================================================
 *
 * 【最常用的接口, 一般只用这几个】
 *
 *   主循环每圈调用一次(必须):
 *     DM4340_Task();            上电检测/自动使能 + 每 10ms 发控制帧, 非阻塞
 *
 *   角度模式: 转到指定角度并锁住
 *     DM4340_SetAngle(90.0f);   转到 90 度(相对上电时的零点, 正负决定方向)
 *     DM4340_IsReached(90.0f, since)   是否已到位(since = 发命令时的 HAL_GetTick())
 *
 *   速度模式: 按指定速度一直转
 *     DM4340_SetSpeed(2.0f);    以 2 rad/s 转(约 19 rpm), 负数反转
 *     DM4340_SetSpeed(0.0f);    减速停下(不锁位置, 外力能推动)
 *
 *   停住并锁在当前位置:
 *     DM4340_Hold();
 *
 *   读状态:
 *     DM4340_GetAngle()         当前角度 (度)
 *     DM4340_GetSpeed()         当前转速 (rad/s)
 *     DM4340_IsOnline()         1 = 电机已上电且已使能
 *
 *   SetAngle / SetSpeed 会自动切换模式, 不用先切模式, 可以每圈都调用。
 *   两种模式都是 MIT 帧, 切换不需要重新使能, 下个控制周期(10ms 内)生效。
 *
 * 【单位】 角度: 度     速度: rad/s (输出轴)    rpm = rad/s * 9.55
 *
 * 【例子】
 *   while (1)
 *   {
 *       DM4340_Task();
 *       if (要转到识别位)  DM4340_SetAngle(-129.15f);
 *       if (要匀速转)      DM4340_SetSpeed(2.0f);
 *   }
 *
 * 【调参】 都在 J4340.c 开头:
 *   角度模式: DM4340_POSITION_KP (刚度), DM4340_POSITION_KD (阻尼)
 *   速度模式: DM4340_SPEED_KD (软/带不动 -> 加大, 抖 -> 减小), DM4340_SPEED_KI (转速偏低 -> 加大),
 *            DM4340_SPEED_ACC (起停太猛 -> 减小), DM4340_SPEED_LIMIT (最高速度)
 * ===================================================================== */

#define DM4340_FIRST_ID       0x01U
#define DM4340_LAST_ID        0x01U   /* 只装了一个达妙电机(ID1), 清错/设零/使能/MIT 都只发给它 */
#define DM4340_FEEDBACK_BASE  0x24U

/* ======================== 最常用接口 ======================== */
void    DM4340_Task(void);                          /* 主循环每圈调用: 上电管理 + 发控制帧 */
void    DM4340_SetAngle(float deg);                 /* 角度模式: 转到 deg 度并锁住 */
void    DM4340_SetSpeed(float rad_s);               /* 速度模式: 以 rad_s 转, 0 = 减速停 */
void    DM4340_Hold(void);                          /* 切角度模式, 锁在当前位置 */
float   DM4340_GetAngle(void);                      /* 当前角度 (度) */
float   DM4340_GetSpeed(void);                      /* 当前转速 (rad/s) */
uint8_t DM4340_IsOnline(void);                      /* 1 = 已上电且已使能 */
uint8_t DM4340_IsReached(float target_deg, uint32_t since); /* 角度是否到位(只认 since 之后的反馈) */

/* ======================== 模式 ======================== */
typedef enum
{
    DM4340_MODE_MIT   = 0,  /* 角度模式(默认) */
    DM4340_MODE_SPEED = 1   /* 速度模式 */
} DM4340_CtrlMode_t;

DM4340_CtrlMode_t DM4340_GetCtrlMode(void);         /* 读当前模式 */

/* ======================== 底层接口 (一般不用直接调) ========================
   SetTargetAngle / SetTargetSpeed 只改目标值, 不切模式; 要配合 SetCtrlMode 使用。
   旧代码(DM_task.c)用的 SetTargetAngle 保留不变。 */
void DM4340_SetCtrlMode(DM4340_CtrlMode_t mode);    /* 切换模式, 下个控制周期生效 */
void DM4340_SetTargetAngle(float angle);            /* 角度模式目标 (度) */
void DM4340_SetTargetSpeed(float speed);            /* 速度模式目标 (rad/s) */
void DM4340_PowerManager(void);                     /* 上电状态机, DM4340_Task 里已调用 */
void DM4340_ControlLoop(void);                      /* 每 10ms 发控制帧, DM4340_Task 里已调用 */

typedef struct
{
    uint8_t id;
    uint8_t state;
    float position;         /* 单位: rad */
    float speed;            /* 单位: rad/s */
    float torque;           /* 单位: N.m */
    uint8_t mos_temperature;
    uint8_t coil_temperature;
    uint32_t rx_tick;       /* 最近一次收到反馈的时刻(ms), 0 表示从没收到过 */
} DM4340_Feedback_t;

typedef struct
{
    uint8_t id;
    float position;
    float speed;
    float kp;
    float kd;
    float torque;
    DM4340_Feedback_t feedback;
} DM4340_Motor_t;

extern DM4340_Motor_t DM4340_Motor[4];
extern volatile uint32_t DM4340_TxDropCnt;          /* FDCAN1 发送失败次数(调试看, 正常一直为 0) */

void DM4340_Init(void);                             /* 初始化电机结构体, 开机调一次 */
void DM4340_ParseFeedback(uint16_t std_id, uint8_t *data); /* FDCAN1 RX FIFO0 接收回调里调用(见 bsp_can_dm.c) */
void DM4340_Send(uint8_t id, uint8_t *data);
void DM4340_Enable(uint8_t id);
void DM4340_Disable(uint8_t id);
void DM4340_EnableAll(void);
void DM4340_DisableAll(void);
void DM4340_ClearErr(uint8_t id);
void DM4340_SetZeroPosition(uint8_t id);
void DM4340_SetMIT(uint8_t id, float position, float speed,
                   float kp, float kd, float torque);
void DM4340_SetMITAll(float position, float speed,
                      float kp, float kd, float torque);
void DM4340_WriteReg(uint8_t id, uint8_t rid, uint32_t value); /* 写电机寄存器(RAM, 断电恢复) */
void DM4340_ReadReg(uint8_t id, uint8_t rid);                  /* 读电机寄存器, 应答在 ParseFeedback 里处理 */

/* 调试变量: 读回的 CTRL_MODE(应为 1) 和 VMAX(rad/s, 0 = 没读到, 用默认 10) */
extern volatile uint32_t DM4340_CtrlModeRead;
extern volatile float    DM4340_VmaxRead;

/* ---------------- 上电检测 + 自动使能 ----------------
   电机上电后自动 清错 -> 设零点(仅第一次) -> 使能;
   运行中掉电/断线, 重新上电后自动再使能。原理和流程详见 J4340.c 里的说明。 */
typedef enum
{
    DM4340_PWR_PROBE = 0,  /* 0: 等电机上电, 每 100ms 发一次失能帧当探测, 收到反馈进入下一步 */
    DM4340_PWR_CLEAR,      /* 1: 已发清错, 等 50ms */
    DM4340_PWR_ZERO,       /* 2: 已设零点(仅第一次上电), 等 50ms 让电机保存 */
    DM4340_PWR_ENABLE,     /* 3: 已发使能, 等反馈确认 state==1; 重试 5 次失败回到 0 */
    DM4340_PWR_RUN         /* 4: 已使能, 正常控制; 300ms 没反馈判掉线, 回到 0 */
} DM4340_PowerState_t;

DM4340_PowerState_t DM4340_GetPowerState(void);     /* 读当前状态(调试用), 值见上面的枚举 */

#endif /* __J4340_H */
