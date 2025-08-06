#include "ameba_soc.h"
#include "os_wrapper.h"
#include <stdio.h>
#include "main.h"
#include "TimeEvent.h"
#include "Lcd.h"
#include "step_motor.h"
#include "task_manager.h"
#include "lvg_lcd_adapter.h"
#include "timer_api.h"
#include "Fan_motor.h"
#include "sen68.h"
//#include "../absolute_encoder/absolute_encoder.h"
#include "absolute_encoder.h"
gtimer_t my_timer1;
/**
 * @brief  timer1 callback function
 */
 static void timer1_callback_handler(void)
{
    // 1ms定时器回调函数
    fw_timer_event_isr_1ms();  // 调用定时器事件处理函数
} 
/**
 * @brief Initialize timer
 */
 static void timer_1ms_init(void)
{
    // Initialize 1ms timer
    gtimer_init(&my_timer1, TIMER1);
	gtimer_start_periodical(&my_timer1, 1000, (void *)timer1_callback_handler, 0);
} 
void test_event_timer_task(void)
{
    printf("Hello world\n");
}
// 创建定时器处理任务
 static void timer_event_task(void *param)
{
    (void)param;  // 避免未使用参数警告
    while(1) {
        fw_timer_event_Handler();  // 处理定时器事件
        rtos_time_delay_ms(1); // 延时1ms
    }
} 
/**
 * @brief  Application example function
 * @param  None
 * @return None
 */
void app_example(void)
{
    printf("hello world\n");
    printf("Starting stepper motor initialization...\n");
    
    fw_timer_event_Init();
        // 创建定时器处理任务
    if (rtos_task_create(NULL, "timer_event_task", timer_event_task, 
                        NULL, 2048, 2) != RTK_SUCCESS) {
      printf("Failed to create timer event task\n");
    }
 //   fan_controller_example();
   // stepper_motor_init();
    timer_1ms_init();
   	fw_timer_event_CancelAllTimerEvent();
   // fw_timer_event_ActiveTimerEvent(10000,sampleing_sen68_data);
   // stepper_motor_set_direction(MOTOR_BASE, Motor_Direction_Forward, 242);
   // stepper_motor_set_speed_profile(MOTOR_BASE, 400, 30);

    //stepper_motor_set_target_position(MOTOR_BASE, 1000);
   // rtos_time_delay_ms(10000);
     //fan_controller_example();
     //fan_speed_controller_init();
    //test_pwm_output();
    //stepper_motor_stop(MOTOR_BASE, false, true);
    // 2. 初始化任务管理系统
    //task_manager_init();
    
   // stepper_motor_set_direction(MOTOR_BASE, Motor_Direction_Forward, 800);
   // DisplayLCD_Init();
   // sen68_test();
    // 3. 启动所有应用任务
   // task_manager_start_all();
  
    printf("All tasks started successfully\n");
 
    printf("LCD display initialized successfully\n");

}