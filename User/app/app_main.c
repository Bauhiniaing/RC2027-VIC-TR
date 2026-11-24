/**
 * @file  app_main.c
 * @brief 测试程序：闭环控制 + 每 0.5 秒打印电机状态到 USB 虚拟串口。
 *
 * 控制在 TIM6 的 1ms 中断里跑（app_1ms_tick），主循环只负责打印。
 * 默认一个电机都不控制、不发任何控制帧。要测试就改 test_start() 里的内容，
 * ⚠ 先把电机架空再测！
 */
#include "app_main.h"
#include "main.h"
#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "motor_cfg.h"
#include "motor_ctrl.h"
#include "lcd.h"
#include "vofa.h"
#include "bsp_crsf.h"
#include <stdio.h>

#define PRINT_PERIOD_MS 500  /* 打印 2Hz */
#define LCD_PERIOD_MS   200  /* 屏幕刷新 5Hz */
#define TLM_PERIOD_MS   500  /* 回传遥控器 2Hz */

/* 1 = 用 VOFA+ 看波形调参（串口发二进制波形，状态表不再打印）；0 = 串口打印状态表 */
#define USE_VOFA        1
#define VOFA_MOTOR      MOTOR_M3508_1  /* 开机默认看哪个电机，VOFA+ 里发 m=<下标> 可以换 */

extern TIM_HandleTypeDef htim6;  /* CubeMX 生成在 main.c，1kHz 控制定时器 */

static volatile uint8_t s_ready;  /* 初始化完成前中断里不做控制 */

/* 在这里写要测试的电机，单位都是输出轴的 */
static void test_start(void)
{
    /* 2006 ID1 接 CAN2：输出轴转到 90°，限速 30 rpm。⚠ 板子要接电池供电，只插 USB 时 CAN 发不出去 */
    // motor_ctrl_set_pos(MOTOR_M2006_1, 90.0f, 30.0f);
    // motor_ctrl_set_speed(MOTOR_M2006_1, 60.0f); 
    /* 3508 调参：开机不动，在 VOFA+ 里发 spd=30 / pos=90 控制。3508 参数是借用 2006 的，先架空！ */
    // motor_ctrl_set_pos(MOTOR_M3508_1, 90.0f, 30.0f);

    /* 例子（去掉注释再用）：
     * motor_ctrl_set_speed(MOTOR_M3508_1, 60.0f);          // 3508 输出轴 60 rpm
     * motor_ctrl_set_pos(MOTOR_M2006_1, 90.0f, 30.0f);     // 2006 输出轴转到 90°，限速 30 rpm
     * motor_ctrl_set_open(MOTOR_M3508_2, 500);             // 直接给 500 电流
     */
}

__attribute__((unused)) static void print_motors(void)
{
    uint32_t now = HAL_GetTick();
    printf("\r\n==== t=%lu ms ====\r\n", (unsigned long)now);

    for (int b = 1; b <= 3; b++) {
        FDCAN_HandleTypeDef *h = (b == 1) ? &hfdcan1 : (b == 2) ? &hfdcan2 : &hfdcan3;
        const bsp_fdcan_stat_t *s = bsp_fdcan_get_stat(h);
        printf("CAN%d rx=%lu tx=%lu tx_fail=%lu busoff=%lu\r\n", b,
               (unsigned long)s->rx_cnt, (unsigned long)s->tx_cnt,
               (unsigned long)s->tx_fail_cnt, (unsigned long)s->bus_off_cnt);
    }

    static const char *const k_mode[] = { "off", "open", "spd", "pos" };
    printf("%-10s %-6s bus id  %-4s %-3s %-4s %5s %6s %6s %4s %10s %9s\r\n",
           "name", "type", "st", "en", "mode", "ecd", "rpm", "cur", "T", "out_deg", "out_rpm");
    for (int i = 0; i < dji_motor_count(); i++) {
        const dji_motor_cfg_t *c = dji_motor_cfg(i);
        const dji_motor_t     *m = dji_motor_get(i);
        if (!dji_motor_is_online(i) && motor_ctrl_mode(i) == MOTOR_MODE_OFF) continue;  /* 没接也没用的不打印 */
        printf("%-10s %-6s  %d  %d  %-4s %-3s %-4s %5u %6d %6d %4u %10.2f %9.2f\r\n",
               c->name, dji_motor_type_name(c->type), bsp_fdcan_index(c->hcan), c->esc_id,
               dji_motor_is_online(i) ? "ON" : "--",
               dji_motor_is_active(i) ? "Y" : "-",
               k_mode[motor_ctrl_mode(i)],
               m->ecd, m->speed_rpm, m->current, m->temperature, m->out_angle_deg, m->out_speed_rpm);
    }
}

/* 屏幕：开机画一次固定内容 */
static void lcd_draw_static(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowChinese(20, 4, "达妙科技", WHITE, BLACK, 32, 0);
    LCD_ShowString(160, 12, (const uint8_t *)"H7_TR", BRRED, BLACK, 24, 0);
    LCD_DrawLine(0, 42, LCD_W - 1, 42, GRAY);
}

/* 屏幕：周期刷新的内容（只重画数字，不清屏，避免闪烁） */
static void lcd_draw_status(void)
{
    uint32_t t = HAL_GetTick() / 1000;
    LCD_Printf(4, 50, WHITE, BLACK, 16, "up %02lu:%02lu:%02lu",
               (unsigned long)(t / 3600), (unsigned long)(t / 60 % 60), (unsigned long)(t % 60));

    for (int b = 1; b <= 3; b++) {
        FDCAN_HandleTypeDef *h = (b == 1) ? &hfdcan1 : (b == 2) ? &hfdcan2 : &hfdcan3;
        const bsp_fdcan_stat_t *s = bsp_fdcan_get_stat(h);
        LCD_Printf(4, 54 + b * 22, s->bus_off_cnt ? RED : GREEN, BLACK, 16,
                   "CAN%d rx%-8lu tx%-8lu", b, (unsigned long)s->rx_cnt, (unsigned long)s->tx_cnt);
    }

    int online = 0;
    for (int i = 0; i < dji_motor_count(); i++) online += dji_motor_is_online(i);
    LCD_Printf(4, 150, online ? GREEN : YELLOW, BLACK, 24, "motor %2d/%-2d", online, dji_motor_count());

    /* 遥控器：连上显示绿色 + 上行链路质量 + 前 4 路通道(us)；失控显示红色 */
    const crsf_link_t *lk = crsf_get_link();
    const crsf_stat_t *st = crsf_get_stat();
    if (crsf_is_connected()) {
        LCD_Printf(4, 180, GREEN, BLACK, 16, "RC OK  LQ%3u%% -%3udBm   ", lk->up_lq, lk->up_rssi1);
    } else {
        LCD_Printf(4, 180, RED, BLACK, 16, "RC LOST rx%-8lu crc%-5lu",
                   (unsigned long)st->rx_bytes, (unsigned long)st->crc_err);
    }
    LCD_Printf(4, 200, WHITE, BLACK, 16, "%4u %4u %4u %4u",
               crsf_get_us(0), crsf_get_us(1), crsf_get_us(2), crsf_get_us(3));
    /* 换算后的摇杆：前推、右推都应该是正数，用来核对方向 */
    rc_sticks_t sk = crsf_get_sticks();
    LCD_Printf(4, 220, CYAN, BLACK, 16, "L%+5.2f %+5.2f R%+5.2f %+5.2f ", sk.lx, sk.ly, sk.rx, sk.ry);
}

void app_init(void)
{
    crsf_init();      /* ELRS 接收机：UART7 PE7/PE8 */
    bsp_fdcan_init();
    int err = dji_motor_init();
    motor_ctrl_init();

    LCD_Init();
    lcd_draw_static();

    HAL_Delay(1500);  /* 等电脑把 USB 串口枚举好，不然开机信息看不到 */
    printf("\r\nH7_TR boot, SYSCLK=%lu MHz\r\n", (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000));
    printf("dji motors in table: %d, config errors: %d\r\n", dji_motor_count(), err);
    for (int i = 0; i < err; i++) {
        const char *e = dji_motor_cfg_error(i);
        if (e) printf("  CFG ERROR: %s\r\n", e);
    }

    vofa_init(VOFA_MOTOR);
    test_start();
    s_ready = 1;

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        printf("TIM6 start failed!\r\n");
        Error_Handler();
    }
}

/* 1ms 调一次（在中断里），F4 里 TIM3 中断那段 */
void app_1ms_tick(void)
{
    if (!s_ready) return;
    motor_ctrl_update();
    vofa_sample_isr();
}

/* HAL 定时器回调，以后其他定时器的中断也写在这里 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        app_1ms_tick();
    }
}

void app_loop(void)
{
    static uint32_t last_print, last_lcd, last_tlm;
    uint32_t now = HAL_GetTick();
    if (now - last_tlm >= TLM_PERIOD_MS) {
        last_tlm = now;
        /* 回传给遥控器，先发个模式字符串验证回传链路；有电压采样后可以加 crsf_send_battery() */
        crsf_send_flight_mode(crsf_is_connected() ? "TR RUN" : "TR IDLE");
    }
    if (now - last_lcd >= LCD_PERIOD_MS) {
        last_lcd = now;
        lcd_draw_status();
    }
#if USE_VOFA
    (void)last_print;
    vofa_poll();
#else
    if (now - last_print >= PRINT_PERIOD_MS) {
        last_print = now;
        print_motors();
    }
#endif
}
