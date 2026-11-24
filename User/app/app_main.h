#ifndef APP_MAIN_H
#define APP_MAIN_H

void app_init(void);  /* 在 main() 的 USER CODE 2 里调用一次 */
void app_loop(void);  /* 在 while(1) 里一直调用 */
void app_1ms_tick(void); /* 1ms 调一次，由 TIM6 中断调用 */

#endif
