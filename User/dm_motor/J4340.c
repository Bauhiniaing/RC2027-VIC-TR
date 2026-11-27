#include "J4340.h"
#include <math.h>
#include <string.h>

/* ------------------ DM4340 MIT 帧量程（必须与电机里的 PMAX/VMAX/TMAX 一致） ------------------ */
#define DM4340_P_MIN      -12.5f     /* 位置下限 (rad) */
#define DM4340_P_MAX       12.5f     /* 位置上限 (rad) */
/* 速度量程不再写死: 出厂 VMAX = 10 rad/s, 之前写的 ±30 和电机不一致,
   导致下发/反馈的速度都差 3 倍。现在上电时从电机读 VMAX 寄存器, 读到就用读到的值,
   读不到用默认 10。位置模式下发的速度是 0(量程中点), 不受影响, 所以以前看不出来 */
#define DM4340_V_MAX_DEFAULT 10.0f
#define DM4340_KP_MIN       0.0f     /* Kp 下限 */
#define DM4340_KP_MAX     500.0f     /* Kp 上限 */
#define DM4340_KD_MIN       0.0f     /* Kd 下限 */
#define DM4340_KD_MAX       5.0f     /* Kd 上限 */
#define DM4340_T_MIN      -28.0f     /* 扭矩下限 (N.m) - 针对4340调大 */
#define DM4340_T_MAX       28.0f     /* 扭矩上限 (N.m) - 针对4340调大 */

#define DM4340_DEG_TO_RAD         0.0174532925f
#define DM4340_CONTROL_PERIOD     10U         /* 控制周期 10ms */
#define DM4340_POSITION_SPEED     0.0f
#define DM4340_POSITION_KP        30.0f
#define DM4340_POSITION_KD        4.0f
#define DM4340_POSITION_TORQUE    0.0f
#define DM4340_REACH_TOL_DEG      2.0f        /* 到位判定: 角度误差(度) */
#define DM4340_REACH_SPEED        0.5f        /* 到位判定: 转速(rad/s), 防止高速冲过目标时误判 */

/* ------------------ 速度模式参数(基于 MIT 帧, 不依赖电机内部速度环, 不用上位机) ------------------
   调参顺序: 先调 KD(转得软/跟不上 -> 加大; 抖/叫 -> 减小), 带载转速偏低再加 KI */
#define DM4340_SPEED_KD           2.0f        /* 速度环 P: 力矩 = KD * (目标速度 - 实际速度), N.m/(rad/s), 0~5 */
#define DM4340_SPEED_KI           4.0f        /* 速度环 I: 补减速箱摩擦造成的稳态误差, N.m/rad; 0 = 纯 P */
#define DM4340_SPEED_I_MAX        5.0f        /* 积分力矩限幅 (N.m), 防止堵转时积分越积越大 */
#define DM4340_SPEED_ACC          20.0f       /* 加速度限制 (rad/s^2): 20 = 0 到 2 rad/s 用 0.1s, 防止起停冲击 */
#define DM4340_SPEED_LIMIT        8.0f        /* 目标速度限幅 (rad/s, 输出轴), 不会超过 VMAX */

/* ------------------ 寄存器读写 (帧 ID 0x7FF) ------------------ */
#define DM4340_REG_CMD_ID         0x7FFU      /* 读写寄存器命令统一发到 0x7FF */
#define DM4340_REG_READ           0x33U       /* 命令字: 0x33 读 */
#define DM4340_REG_WRITE          0x55U       /* 命令字: 0x55 写 */
#define DM4340_REG_SAVE           0xAAU       /* 命令字: 0xAA 保存到 flash (本程序不用) */
#define DM4340_REG_CTRL_MODE      0x0AU       /* RID 10: CTRL_MODE 控制模式 1=MIT 2=位置速度 3=速度 4=力位混合 */
#define DM4340_REG_VMAX           0x16U       /* RID 22: VMAX, MIT 帧速度量程 (float) */
#define DM4340_CTRL_MODE_MIT      1U
#define DM4340_SPEEDMODE_ID_OFF   0x200U      /* 原生速度模式的帧 ID 偏移; 只在探测/清错时用, 见 PowerManager */

DM4340_Motor_t DM4340_Motor[4];
static float DM4340_TargetAngle = 0.0f;
static float DM4340_TargetSpeed = 0.0f;                            /* 速度模式目标速度 (rad/s) */
static volatile DM4340_CtrlMode_t dm_ctrl_mode = DM4340_MODE_MIT;  /* 当前控制模式, 默认角度(MIT)模式 */
static float dm_v_max = DM4340_V_MAX_DEFAULT;                      /* MIT 速度量程 ±dm_v_max (rad/s) */

static float   dm_speed_ramp = 0.0f;      /* 斜坡后的实际下发速度 (rad/s), 每周期向目标逼近 */
static float   dm_speed_integral = 0.0f;  /* 积分力矩 (N.m), 作为 MIT 前馈力矩 t_ff 下发 */
static uint8_t dm_speed_reset = 1U;       /* 1 = 下个周期重新初始化斜坡和积分 */

/* 调试变量 (Keil Watch 里看):
   DM4340_CtrlModeRead : 读回的 CTRL_MODE, 正常 = 1 (MIT); 0xFF = 没读到
   DM4340_VmaxRead     : 读回的 VMAX (rad/s); 0 = 没读到, 此时用默认 10 */
volatile uint32_t DM4340_CtrlModeRead = 0xFFU;
volatile float    DM4340_VmaxRead = 0.0f;

static float DM4340_Clamp(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* 数据类型映射转换 */
static uint16_t DM4340_FloatToUint(float value, float min, float max, uint8_t bits)
{
    float span = max - min;
    uint32_t range = (1UL << bits) - 1UL;

    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }

    return (uint16_t)((value - min) * range / span);
}

static float DM4340_UintToFloat(uint16_t value, float min, float max, uint8_t bits)
{
    float span = max - min;
    uint32_t range = (1UL << bits) - 1UL;

    return (float)value * span / range + min;
}

static uint8_t DM4340_CheckID(uint8_t id)
{
    return id >= DM4340_FIRST_ID && id <= DM4340_LAST_ID;
}

/* 用 FDCAN1 发送报文 (经 bsp_fdcan 发送, 经典 CAN 帧, 不开 BRS) */
volatile uint32_t DM4340_TxDropCnt = 0; // FDCAN1 发送失败次数

/* 通用发送: 任意标准帧 ID (MIT 帧发到 ID, 寄存器帧发到 0x7FF) */
static void DM4340_SendRaw(uint16_t std_id, uint8_t *data, uint8_t len)
{
    /* H7 FDCAN 的 TX FIFO 有 8 个元素(TxFifoQueueElmtsNbr = 8), 比 bxCAN 的 3 个邮箱多;
       连发几帧时仍等 FIFO 有空位(1Mbps 一帧约 0.13ms), 最多等 2ms */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U && (HAL_GetTick() - t0) < 2U)
    {
    }

    if (bsp_fdcan_send_std(&hfdcan1, std_id, data, len) != 0)
    {
        DM4340_TxDropCnt++; // 调试看: 正常应一直为 0
    }
}

/* 原接口保留: 帧 ID = id, 8 字节(MIT 帧用) */
void DM4340_Send(uint8_t id, uint8_t *data)
{
    DM4340_SendRaw(id, data, 8U);
}

void DM4340_Init(void)
{
    uint8_t id;

    for (id = DM4340_FIRST_ID; id <= DM4340_LAST_ID; id++)
    {
        DM4340_Motor[id - DM4340_FIRST_ID].id = id;
        DM4340_Motor[id - DM4340_FIRST_ID].position = 0.0f;
        DM4340_Motor[id - DM4340_FIRST_ID].speed = 0.0f;
        DM4340_Motor[id - DM4340_FIRST_ID].kp = 0.0f;
        DM4340_Motor[id - DM4340_FIRST_ID].kd = 0.0f;
        DM4340_Motor[id - DM4340_FIRST_ID].torque = 0.0f;
    }
}

/* 使能电机 */
void DM4340_Enable(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    if (DM4340_CheckID(id))
    {
        DM4340_Send(id, data);
    }
}

/* 失能电机 */
void DM4340_Disable(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    if (DM4340_CheckID(id))
    {
        DM4340_Send(id, data);
    }
}

void DM4340_EnableAll(void)
{
    for (uint8_t id = DM4340_FIRST_ID; id <= DM4340_LAST_ID; id++)
    {
        DM4340_Enable(id);
    }
}

void DM4340_DisableAll(void)
{
    for (uint8_t id = DM4340_FIRST_ID; id <= DM4340_LAST_ID; id++)
    {
        DM4340_Disable(id);
    }
}

void DM4340_SetTargetAngle(float angle)
{
    DM4340_TargetAngle = angle;
}

/* 速度模式一个控制周期: 斜坡 + PI, 用 MIT 帧发出去。
   MIT 力矩公式: T = kp*(p_des - p) + kd*(v_des - v) + t_ff
   令 kp = 0 去掉位置环, kd 就是速度环的 P(在电机内部高频执行, 响应快);
   主控每 10ms 把速度误差积分成力矩, 通过 t_ff 下发, 就是速度环的 I。
   说明书 MIT 模式一节: "kp=0, kd≠0 时给定 v_des 即可实现匀速转动" */
static void DM4340_SpeedStep(void)
{
    const float dt = DM4340_CONTROL_PERIOD * 0.001f;
    const float max_step = DM4340_SPEED_ACC * dt;  /* 每周期速度最多变化量 */
    const DM4340_Feedback_t *fb = &DM4340_Motor[0].feedback;
    float limit = (DM4340_SPEED_LIMIT < dm_v_max) ? DM4340_SPEED_LIMIT : dm_v_max;
    float target = DM4340_Clamp(DM4340_TargetSpeed, -limit, limit);

    /* 刚进速度模式 / 刚重新上线: 斜坡从当前实际速度起步, 积分清零, 不会突变 */
    if (dm_speed_reset != 0U)
    {
        dm_speed_ramp = DM4340_Clamp(fb->speed, -limit, limit);
        dm_speed_integral = 0.0f;
        dm_speed_reset = 0U;
    }

    /* 1. 斜坡: 下发速度每周期最多向目标靠近 max_step */
    dm_speed_ramp += DM4340_Clamp(target - dm_speed_ramp, -max_step, max_step);

    /* 2. 积分: 目标为 0 且已减速完时清掉积分, 停下后不会被残留力矩推着走 */
    if (target == 0.0f && fabsf(dm_speed_ramp) < 1e-3f)
    {
        dm_speed_integral = 0.0f;
    }
    else
    {
        dm_speed_integral += DM4340_SPEED_KI * (dm_speed_ramp - fb->speed) * dt;
        dm_speed_integral = DM4340_Clamp(dm_speed_integral, -DM4340_SPEED_I_MAX, DM4340_SPEED_I_MAX);
    }

    /* 3. 发 MIT 帧: p_des 填 0(kp=0 时不起作用), v_des = 斜坡速度, t_ff = 积分力矩 */
    DM4340_SetMITAll(0.0f, dm_speed_ramp, 0.0f, DM4340_SPEED_KD, dm_speed_integral);
}

void DM4340_ControlLoop(void)
{
    static uint32_t last_tick = 0;
    float target_position;

    if (HAL_GetTick() - last_tick >= DM4340_CONTROL_PERIOD)
    {
        /* 隔了很久才被调用(电机掉线期间 main 不调本函数), 速度环要重新初始化 */
        if (HAL_GetTick() - last_tick > 5U * DM4340_CONTROL_PERIOD)
        {
            dm_speed_reset = 1U;
        }
        last_tick = HAL_GetTick();

        if (dm_ctrl_mode == DM4340_MODE_SPEED)
        {
            DM4340_SpeedStep();
            return;
        }

        target_position = DM4340_TargetAngle * DM4340_DEG_TO_RAD;

        DM4340_SetMITAll(target_position,
                         DM4340_POSITION_SPEED,
                         DM4340_POSITION_KP,
                         DM4340_POSITION_KD,
                         DM4340_POSITION_TORQUE);
    }
}

/* MIT 命令组包并发送 */
void DM4340_SetMIT(uint8_t id, float position, float speed,
                   float kp, float kd, float torque)
{
    uint16_t p, v, kp_val, kd_val, t;
    uint8_t data[8];
    DM4340_Motor_t *motor;

    if (!DM4340_CheckID(id))
    {
        return;
    }

    motor = &DM4340_Motor[id - DM4340_FIRST_ID];
    motor->position = position;
    motor->speed = speed;
    motor->kp = kp;
    motor->kd = kd;
    motor->torque = torque;

    p = DM4340_FloatToUint(position, DM4340_P_MIN, DM4340_P_MAX, 16);
    v = DM4340_FloatToUint(speed, -dm_v_max, dm_v_max, 12);
    kp_val = DM4340_FloatToUint(kp, DM4340_KP_MIN, DM4340_KP_MAX, 12);
    kd_val = DM4340_FloatToUint(kd, DM4340_KD_MIN, DM4340_KD_MAX, 12);
    t = DM4340_FloatToUint(torque, DM4340_T_MIN, DM4340_T_MAX, 12);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (kp_val >> 8));
    data[4] = (uint8_t)kp_val;
    data[5] = (uint8_t)(kd_val >> 4);
    data[6] = (uint8_t)((kd_val << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    DM4340_Send(id, data);
}

void DM4340_SetMITAll(float position, float speed,
                       float kp, float kd, float torque)
{
    for (uint8_t id = DM4340_FIRST_ID; id <= DM4340_LAST_ID; id++)
    {
        DM4340_SetMIT(id, position, speed, kp, kd, torque);
    }
}

/* 反馈数据解析 */
void DM4340_ParseFeedback(uint16_t std_id, uint8_t *data)
{
    uint8_t id;
    uint16_t position, speed, torque;
    DM4340_Feedback_t *feedback;

    if (std_id >= DM4340_FEEDBACK_BASE && std_id <= DM4340_FEEDBACK_BASE + 3U)
    {
        id = (uint8_t)(std_id - DM4340_FEEDBACK_BASE + 1U);
    }
    else
    {
        /* 达妙反馈帧 D[0] 低 4 位就是电机 CAN ID: 上位机里的 MST_ID 和 DM4340_FEEDBACK_BASE 不一致时也能对上 */
        id = data[0] & 0x0F;
    }
    if (!DM4340_CheckID(id))
    {
        return;
    }
    /* 写寄存器的应答帧也是从 MST_ID 发回来的, 格式 [CANID_L, CANID_H, 0x33/0x55/0xAA, RID, 数据...],
       不是电机状态反馈, 不能当反馈解析(否则位置/状态会被写成乱值) */
    if (data[0] == id && data[1] == 0x00U &&
        (data[2] == DM4340_REG_READ || data[2] == DM4340_REG_WRITE || data[2] == DM4340_REG_SAVE))
    {
        if (data[2] != DM4340_REG_SAVE)
        {
            if (data[3] == DM4340_REG_CTRL_MODE)
            {
                /* D[4..7] = uint32 小端 */
                DM4340_CtrlModeRead = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                                      ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
            }
            else if (data[3] == DM4340_REG_VMAX)
            {
                /* D[4..7] = float 小端; 数值合理才采用, 防止读到乱值把速度量程搞坏 */
                float vmax;
                memcpy(&vmax, &data[4], sizeof(vmax));
                DM4340_VmaxRead = vmax;
                if (vmax > 1.0f && vmax <= 200.0f)
                {
                    dm_v_max = vmax;
                }
            }
        }
        return;
    }
    feedback = &DM4340_Motor[id - DM4340_FIRST_ID].feedback;

    position = ((uint16_t)data[1] << 8) | data[2];
    speed = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];

    feedback->id = data[0] & 0x0F;
    feedback->state = data[0] >> 4;
    feedback->position = DM4340_UintToFloat(position, DM4340_P_MIN, DM4340_P_MAX, 16);
    feedback->speed = DM4340_UintToFloat(speed, -dm_v_max, dm_v_max, 12);
    feedback->torque = DM4340_UintToFloat(torque, DM4340_T_MIN, DM4340_T_MAX, 12);
    feedback->mos_temperature = data[6];
    feedback->coil_temperature = data[7];
    feedback->rx_tick = HAL_GetTick();
}
/* 云台是否已转到目标角度: 只认 since 之后收到的反馈(防止用转动前的旧数据),
   角度误差和转速都足够小才算到位。收不到反馈时永远返回 0, 调用方要配超时 */
uint8_t DM4340_IsReached(float target_deg, uint32_t since)
{
    const DM4340_Feedback_t *fb = &DM4340_Motor[0].feedback;
    float err_deg;

    if ((int32_t)(fb->rx_tick - since) <= 0)
    {
        return 0U;
    }
    err_deg = fb->position / DM4340_DEG_TO_RAD - target_deg;
    return ((fabsf(err_deg) < DM4340_REACH_TOL_DEG) && (fabsf(fb->speed) < DM4340_REACH_SPEED)) ? 1U : 0U;
}

/* 将当前位置设置为电机零点 (0xFE) */
void DM4340_SetZeroPosition(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

    if (DM4340_CheckID(id))
    {
        DM4340_Send(id, data);
    }
}
void DM4340_ClearErr(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

    if (DM4340_CheckID(id))
    {
        DM4340_Send(id, data);
    }
}

/* =====================================================================
 *                 上电检测 + 自动使能 (非阻塞状态机)
 * =====================================================================
 * 【为什么要这个】
 *   原来的 DM4340_PositionInit() 开机只发一次 清错/设零/使能。
 *   如果电机比主控晚上电(比如电机电源后接), 这几帧发出去时电机还没在线,
 *   没人 ACK; (F4 上 CAN1 配置了 AutoRetransmission = DISABLE, 帧就直接丢了;
 *   H7 FDCAN1 现在是 ENABLE, 但电机没上电时一样收不到),
 *   电机上电后一直处于失能状态, 不会动。
 *
 * 【原理】
 *   达妙电机收到任何一帧命令都会回一帧反馈(反馈在 FDCAN1 RX FIFO0 中断里由
 *   DM4340_ParseFeedback() 解析, 并记录收到的时刻 feedback.rx_tick)。
 *   所以周期性发一帧"失能"命令当探测, 一旦收到反馈就说明电机已上电。
 *   用"失能"帧探测是安全的: 电机即使在线也不会动。
 *
 * 【流程】
 *   PROBE  每 100ms 发一次探测帧, 等反馈
 *     |    收到反馈(电机已上电)
 *     v
 *   CLEAR  发清错 0xFB, 等 50ms
 *     v    写 CTRL_MODE = 1 (MIT), 读回 CTRL_MODE 和 VMAX
 *   ZERO   (仅第一次上电)发设零 0xFE, 等 50ms 让电机保存零点
 *     v
 *   ENABLE 发使能 0xFC, 50ms 后检查反馈 state 是否 = 1(已使能)
 *     |    不是 -> 再清错+使能, 最多重试 5 次; 还不行 -> 回 PROBE
 *     v
 *   RUN    正常运行, 主循环里 DM4340_ControlLoop() 每 10ms 发 MIT 帧
 *          超过 300ms 收不到反馈(断电/断线) -> 回 PROBE, 重新上电后自动再使能
 *
 * 【用法】
 *   主循环里每圈调用 DM4340_PowerManager();
 *   只有 DM4340_IsOnline() 为 1 时才调用 DM4340_ControlLoop() 发 MIT 帧。
 *   整个过程不用 HAL_Delay, 不会卡住主循环里的其他任务。
 *
 * 【调试】 在调试器里看 dm_pwr_state:
 *   一直是 0(PROBE)  -> 收不到反馈: 查电机供电、CAN 线、波特率、电机 ID
 *   卡在 3(ENABLE)   -> 有反馈但使能不上: 看 DM4340_Motor[0].feedback.state 故障码
 *   稳定在 4(RUN)    -> 正常
 * ===================================================================== */
#define DM4340_PROBE_PERIOD_MS   100U  /* 探测帧周期(ms): 等上电期间多久发一次失能帧 */
#define DM4340_STEP_DELAY_MS      50U  /* 清错/设零/使能 每一步之间的间隔(ms), 给电机处理时间 */
#define DM4340_OFFLINE_MS        300U  /* 掉线判定(ms): 运行中这么久没收到反馈就认为掉电/断线 */
#define DM4340_ENABLE_RETRY_MAX    5U  /* 使能后没确认成功时的重试次数, 用完回到探测状态 */
#define DM4340_STATE_ENABLED       1U  /* 反馈帧 D[0] 高 4 位的状态码: 0=失能 1=使能 8~F=各种故障 */

/* 状态机当前状态; volatile 方便在调试器里实时观察 */
static volatile DM4340_PowerState_t dm_pwr_state = DM4340_PWR_PROBE;
/* 进入当前状态(或上次发帧)的时刻, 用来算每一步已经等了多久 */
static uint32_t dm_pwr_tick = 0;
/* 本轮探测的开始时刻: 只认这之后收到的反馈, 防止把掉线前的旧反馈当成"已上电" */
static uint32_t dm_probe_since = 0;
/* 零点是否已经设过: 只在第一次上电设零。中途掉电重连不能再设,
   否则会把掉电那一刻云台停的位置当成新零点, 之后所有角度都会偏 */
static uint8_t  dm_zero_done = 0U;
/* ENABLE 状态下已经重试使能的次数 */
static uint8_t  dm_enable_retry = 0U;

/* 判断 since 时刻之后(含)有没有收到过电机反馈。
   rx_tick == 0 表示从没收到过任何反馈。
   用有符号差值比较, HAL_GetTick() 约 49.7 天溢出回绕时也不会判错。 */
static uint8_t DM4340_FeedbackAfter(uint32_t since)
{
    uint32_t rx = DM4340_Motor[0].feedback.rx_tick;
    return (rx != 0U) && ((int32_t)(rx - since) >= 0);
}

/* 切换到新状态, 并把计时起点设为现在 */
static void DM4340_PwrGoto(DM4340_PowerState_t s)
{
    dm_pwr_state = s;
    dm_pwr_tick = HAL_GetTick();
}

/* 上电检测 + 自动使能状态机, 主循环里每圈调用一次。
   非阻塞: 每次调用只检查一下时间/反馈, 该发帧就发一帧, 然后立刻返回 */
void DM4340_PowerManager(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - dm_pwr_tick; /* 在当前状态已经待了多久(ms) */

    switch (dm_pwr_state)
    {
    /* ---- 0. 等电机上电: 周期发探测帧, 收到反馈就开始初始化 ---- */
    case DM4340_PWR_PROBE:
        if (DM4340_FeedbackAfter(dm_probe_since))
        {
            /* 电机回反馈了 = 已上电; 第一步先清错(上电/掉线后可能残留故障)。
               电机可能还停在上一版程序写进 RAM 的原生速度模式, 两组 ID 都清一次 */
            uint8_t clr[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
            DM4340_SendRaw(DM4340_FIRST_ID, clr, 8U);
            DM4340_SendRaw((uint16_t)(DM4340_FIRST_ID + DM4340_SPEEDMODE_ID_OFF), clr, 8U);
            DM4340_PwrGoto(DM4340_PWR_CLEAR);
        }
        else if (dt >= DM4340_PROBE_PERIOD_MS)
        {
            /* 探测帧: 发失能是安全的, 电机在线就会回反馈。
               上一版程序会把电机切到原生速度模式(只认 0x200+ID), 电机没断电的话还在那个模式,
               所以 ID 和 0x200+ID 各发一帧, 哪种情况都能探测到 */
            uint8_t probe[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
            DM4340_SendRaw(DM4340_FIRST_ID, probe, 8U);
            DM4340_SendRaw((uint16_t)(DM4340_FIRST_ID + DM4340_SPEEDMODE_ID_OFF), probe, 8U);
            dm_pwr_tick = now;               /* 只重置计时, 不换状态 */
        }
        break;

    /* ---- 1. 已清错, 等 50ms 后设零点(只有第一次上电才设) ---- */
    case DM4340_PWR_CLEAR:
        if (dt >= DM4340_STEP_DELAY_MS)
        {
            /* 角度和速度都用 MIT 帧, 所以电机固定用 MIT 模式。电机此时失能, 写模式寄存器
               (只写 RAM)确保它是 MIT, 把上一版程序切过去的原生速度模式改回来 */
            DM4340_WriteReg(DM4340_FIRST_ID, DM4340_REG_CTRL_MODE, DM4340_CTRL_MODE_MIT);
            if (dm_zero_done == 0U)
            {
                DM4340_SetZeroPosition(DM4340_FIRST_ID); /* 把当前位置设为 0 度 */
                dm_zero_done = 1U;
            }
            DM4340_PwrGoto(DM4340_PWR_ZERO);
        }
        break;

    /* ---- 2. 等 50ms 让电机保存零点, 然后发使能 ---- */
    case DM4340_PWR_ZERO:
        if (dt >= DM4340_STEP_DELAY_MS)
        {
            /* 读回控制模式(应为 1)和速度量程 VMAX, 结果见 DM4340_CtrlModeRead / DM4340_VmaxRead */
            DM4340_ReadReg(DM4340_FIRST_ID, DM4340_REG_CTRL_MODE);
            DM4340_ReadReg(DM4340_FIRST_ID, DM4340_REG_VMAX);
            DM4340_EnableAll();
            dm_enable_retry = 0U;
            DM4340_PwrGoto(DM4340_PWR_ENABLE);
        }
        break;

    /* ---- 3. 已发使能, 50ms 后看反馈确认是否真的使能了 ---- */
    case DM4340_PWR_ENABLE:
        if (dt >= DM4340_STEP_DELAY_MS)
        {
            /* 必须是发使能之后收到的反馈, 而且状态码 = 1(已使能) */
            if (DM4340_FeedbackAfter(dm_pwr_tick) &&
                DM4340_Motor[0].feedback.state == DM4340_STATE_ENABLED)
            {
                DM4340_PwrGoto(DM4340_PWR_RUN); /* 使能成功, 进入正常运行 */
            }
            else if (++dm_enable_retry <= DM4340_ENABLE_RETRY_MAX)
            {
                /* 没使能上(可能有故障): 再清一次错 + 使能, 再等 50ms 检查 */
                DM4340_ClearErr(DM4340_FIRST_ID);
                DM4340_EnableAll();
                dm_pwr_tick = now;
            }
            else
            {
                /* 重试用完还不行: 回到探测状态, 从头再来 */
                dm_probe_since = now;
                DM4340_PwrGoto(DM4340_PWR_PROBE);
            }
        }
        break;

    /* ---- 4. 正常运行: 这里只负责监视掉线, MIT 帧由 DM4340_ControlLoop() 发 ---- */
    case DM4340_PWR_RUN:
    default:
        /* MIT 帧每 10ms 发一次, 每帧电机都会回反馈; 超过 300ms 没反馈 = 掉电或断线。
           必须用有符号比较: 反馈中断可能在取 now 之后才更新 rx_tick, 使 rx_tick 比 now 新 1ms,
           无符号相减会得到一个巨大的数, 被误判成掉线 */
        if ((int32_t)(now - DM4340_Motor[0].feedback.rx_tick) >= (int32_t)DM4340_OFFLINE_MS)
        {
            /* 回到探测状态, 只认从现在开始的新反馈; 电机重新上电后会自动再走一遍初始化 */
            dm_speed_reset = 1U; /* 速度模式: 重连后斜坡从实际速度(约 0)重新起步, 积分清零, 不会突然冲出去 */
            dm_probe_since = now;
            DM4340_PwrGoto(DM4340_PWR_PROBE);
        }
        break;
    }
}

/* 读取状态机当前状态(调试/显示用), 取值见 J4340.h 里的 DM4340_PowerState_t */
DM4340_PowerState_t DM4340_GetPowerState(void)
{
    return dm_pwr_state;
}

/* 电机是否在线且已使能: 1 = 可以发 MIT 控制帧; 0 = 还没上电 / 正在初始化 / 已掉线 */
uint8_t DM4340_IsOnline(void)
{
    return (dm_pwr_state == DM4340_PWR_RUN) ? 1U : 0U;
}

/* =====================================================================
 *                    速度模式 (基于 MIT 帧)
 * =====================================================================
 * 【为什么不用电机的原生速度模式(0x200+ID)】
 *   原生速度模式用的是电机内部速度环 KP_ASR/KI_ASR, 这组参数要在上位机里设
 *   阻尼系数(推荐 4.0)自动计算; 没设好时速度环振荡 -> "电机里面震, 轴不转"。
 *   MIT 模式在这台电机上已验证可用(角度模式好使), 所以速度模式也走 MIT 帧,
 *   速度环在 DM4340_SpeedStep() 里: kp=0, kd 当 P, 主控积分当 I。
 *
 * 【用法】
 *   DM4340_SetCtrlMode(DM4340_MODE_SPEED);   切到速度模式(开机或运行中都行, 不会重新使能)
 *   DM4340_SetTargetSpeed(2.0f);             输出轴 2 rad/s, 正负决定方向, 0 = 减速到停
 *   DM4340_SetCtrlMode(DM4340_MODE_MIT);     切回角度模式: 原地锁住当前位置, 不会跳回旧目标角
 *   主循环照旧: PowerManager() + IsOnline() 时 ControlLoop()
 *
 * 【注意】
 *   - 目标速度限幅 ±DM4340_SPEED_LIMIT(且不超过 VMAX), 加速度限制 DM4340_SPEED_ACC;
 *   - 速度模式下目标为 0 时只是"停", 不锁位置, 外力可以推动; 要锁位置请切回角度模式。
 * ===================================================================== */

/* 写寄存器: 0x7FF, [CANID_L, CANID_H, 0x55, RID, 值(uint32 小端)]; 只写 RAM, 断电恢复 */
void DM4340_WriteReg(uint8_t id, uint8_t rid, uint32_t value)
{
    uint8_t data[8];

    if (!DM4340_CheckID(id))
    {
        return;
    }
    data[0] = id;
    data[1] = 0x00U;
    data[2] = DM4340_REG_WRITE;
    data[3] = rid;
    data[4] = (uint8_t)(value);
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);
    DM4340_SendRaw(DM4340_REG_CMD_ID, data, 8U);
}

/* 读寄存器: 0x7FF, [CANID_L, CANID_H, 0x33, RID]; 应答由 ParseFeedback 处理 */
void DM4340_ReadReg(uint8_t id, uint8_t rid)
{
    uint8_t data[8] = {0};

    if (!DM4340_CheckID(id))
    {
        return;
    }
    data[0] = id;
    data[1] = 0x00U;
    data[2] = DM4340_REG_READ;
    data[3] = rid;
    DM4340_SendRaw(DM4340_REG_CMD_ID, data, 8U);
}

/* 设目标速度 (rad/s, 输出轴); rpm 约等于 rad/s * 9.55。只记录目标, 由 ControlLoop 每 10ms 发出 */
void DM4340_SetTargetSpeed(float speed)
{
    DM4340_TargetSpeed = speed;
}

/* 切换控制模式。两种模式都是 MIT 帧, 电机不用失能/重新使能, 下个控制周期立即生效 */
void DM4340_SetCtrlMode(DM4340_CtrlMode_t mode)
{
    if (mode == dm_ctrl_mode)
    {
        return;
    }
    if (mode == DM4340_MODE_SPEED)
    {
        DM4340_TargetSpeed = 0.0f;   /* 切过来先不动, 等上层给目标 */
        dm_speed_reset = 1U;         /* 斜坡从当前实际速度起步, 积分清零 */
    }
    else
    {
        /* 切回角度模式: 目标角设成当前位置, 原地锁住, 不会往旧的目标角猛跳 */
        DM4340_TargetAngle = DM4340_Motor[0].feedback.position / DM4340_DEG_TO_RAD;
    }
    dm_ctrl_mode = mode;
}

DM4340_CtrlMode_t DM4340_GetCtrlMode(void)
{
    return dm_ctrl_mode;
}

/* =====================================================================
 *                     最常用接口 (见 J4340.h 顶部说明)
 * ===================================================================== */

/* 主循环每圈调用一次: 上电检测/自动使能 + 电机在线时每 10ms 发一帧控制命令。非阻塞 */
void DM4340_Task(void)
{
    DM4340_PowerManager();
    if (DM4340_IsOnline())
    {
        DM4340_ControlLoop();
    }
}

/* 角度模式: 转到 deg 度(相对上电零点)并锁住。当前是速度模式会自动切过来 */
void DM4340_SetAngle(float deg)
{
    DM4340_SetCtrlMode(DM4340_MODE_MIT);
    DM4340_TargetAngle = deg;
}

/* 速度模式: 以 rad_s 转(输出轴, 负数反转, 0 = 减速停)。当前是角度模式会自动切过来 */
void DM4340_SetSpeed(float rad_s)
{
    DM4340_SetCtrlMode(DM4340_MODE_SPEED);
    DM4340_TargetSpeed = rad_s;
}

/* 停住并锁在当前位置: 切到角度模式, 目标角 = 当前角度。
   已经在角度模式时也会把目标改成当前位置(即放弃原来的目标角, 原地停) */
void DM4340_Hold(void)
{
    DM4340_SetCtrlMode(DM4340_MODE_MIT);
    DM4340_TargetAngle = DM4340_GetAngle();
}

/* 当前角度 (度), 来自电机反馈 */
float DM4340_GetAngle(void)
{
    return DM4340_Motor[0].feedback.position / DM4340_DEG_TO_RAD;
}

/* 当前转速 (rad/s, 输出轴), 来自电机反馈 */
float DM4340_GetSpeed(void)
{
    return DM4340_Motor[0].feedback.speed;
}
