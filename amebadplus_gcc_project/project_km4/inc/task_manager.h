#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#include "ameba_soc.h"
#include "os_wrapper.h"

/*================================ 任务优先级定义 ==============================*/
#define KEY_TASK_PRIORITY        2    // 按键任务优先级最高
#define MOTOR_TASK_PRIORITY      3    // 电机控制任务中等优先级  
#define LCD_TASK_PRIORITY        5    // LCD刷新任务优先级较低

/*================================ 任务堆栈大小 ==============================*/
#define KEY_TASK_STACK_SIZE      1024   // 按键任务堆栈
#define MOTOR_TASK_STACK_SIZE    2048   // 电机任务堆栈
#define LCD_TASK_STACK_SIZE      2048   // LCD任务堆栈

/*================================ 任务状态控制 ==============================*/
typedef enum {
    TASK_STATE_STOP = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_SUSPEND
} task_state_t;

/*================================ 全局变量声明 ==============================*/
extern rtos_task_t key_task_handle;
extern rtos_task_t motor_task_handle;
extern rtos_task_t lcd_task_handle;

extern volatile uint8_t lcd_update_flag;
extern volatile uint8_t motor_control_flag;
extern volatile uint8_t system_power_state;

/*================================ 函数声明 ==============================*/
// 任务管理函数
void task_manager_init(void);
void task_manager_start_all(void);
void task_manager_stop_all(void);

// 各个任务函数
void key_scan_task(void *param);
void motor_control_task(void *param);
void lcd_display_task(void *param);

// 任务间通信函数
void set_lcd_update_flag(void);
void set_motor_control_flag(uint8_t direction);
void set_system_power_state(uint8_t state);

#endif // __TASK_MANAGER_H__