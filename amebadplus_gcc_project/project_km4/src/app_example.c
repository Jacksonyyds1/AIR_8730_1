#include "ameba_soc.h"
#include "os_wrapper.h"
#include <stdio.h>
#include "main.h"
#include "TimeEvent.h"
#include "Lcd.h"
#include "step_motor.h"
#include "task_manager.h"

/*===================================  Types ================================*/
typedef struct {
    void (*event_callback)(void);
    uint16_t time_remaining;
    uint16_t interval_ms;
    uint8_t is_active;
} timer_event_t;

/*================================ Definitions ==============================*/
#define MAX_TIMER_EVENTS            12
#define TIMER_EVENT_TASK_PRIORITY   2
#define TIMER_EVENT_STACK_SIZE      256
#define TIMER_TICK_INTERVAL_MS      10  // 10ms精度

/*================================== Variables ==============================*/
static timer_event_t timer_events[MAX_TIMER_EVENTS];
static rtos_mutex_t timer_event_mutex = NULL;
static rtos_timer_t system_timer = NULL;
static uint8_t system_initialized = 0;

/*============================================================================*/
/**
 * 系统定时器回调函数 - 每10ms执行一次
 */
static void system_timer_callback(void *arg)
{
    (void)arg;
    uint8_t i;
    
    // 遍历所有定时器事件
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        if (timer_events[i].is_active && timer_events[i].time_remaining > 0) {
            timer_events[i].time_remaining--;
            
            // 时间到了，执行回调
            if (timer_events[i].time_remaining == 0) {
                if (timer_events[i].event_callback != NULL) {
                    timer_events[i].event_callback();
                }
                // 重新装载时间
                timer_events[i].time_remaining = timer_events[i].interval_ms / TIMER_TICK_INTERVAL_MS;
            }
        }
    }
}

/*============================================================================*/
/**
 * 初始化定时器事件系统
 */
void fw_timer_event_init(void)
{
    uint8_t i;
    
    if (system_initialized) {
        return; // 已经初始化过了
    }
    
    // 创建互斥锁
    if (rtos_mutex_create(&timer_event_mutex) != RTK_SUCCESS) {
        printf("Failed to create timer event mutex!\n");
        return;
    }
    
    // 初始化定时器事件数组
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        timer_events[i].event_callback = NULL;
        timer_events[i].time_remaining = 0;
        timer_events[i].interval_ms = 0;
        timer_events[i].is_active = 0;
    }
    
    // 创建系统定时器，每10ms触发一次
    if (rtos_timer_create(&system_timer,
                         "SysTimer",
                         0,                           // timer_id
                         TIMER_TICK_INTERVAL_MS,      // 10ms间隔
                         1,                           // 周期性
                         system_timer_callback       // 回调函数
                         ) == RTK_SUCCESS) {
        
        // 启动系统定时器
        rtos_timer_start(system_timer, 0);
        system_initialized = 1;
        printf("Timer event system initialized successfully\n");
    } else {
        printf("Failed to create system timer!\n");
    }
}

/*============================================================================*/
/**
 * 反初始化定时器事件系统
 */
void fw_timer_event_deinit(void)
{
    uint8_t i;
    
    if (!system_initialized) {
        return;
    }
    
    if (timer_event_mutex != NULL) {
        rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
        
        // 停止并删除系统定时器
        if (system_timer != NULL) {
            rtos_timer_stop(system_timer, 0);
            rtos_timer_delete(system_timer, 0);
            system_timer = NULL;
        }
        
        // 清除所有事件
        for (i = 0; i < MAX_TIMER_EVENTS; i++) {
            timer_events[i].event_callback = NULL;
            timer_events[i].time_remaining = 0;
            timer_events[i].interval_ms = 0;
            timer_events[i].is_active = 0;
        }
        
        rtos_mutex_give(timer_event_mutex);
        rtos_mutex_delete(timer_event_mutex);
        timer_event_mutex = NULL;
        system_initialized = 0;
    }
}

/*============================================================================*/
/**
 * 激活定时器事件
 */
uint8_t fw_timer_event_active(uint16_t ms_time, void (*event_func)(void))
{
    uint8_t i;
    uint8_t result = _FALSE;
    uint16_t timer_ticks;
    
    if (event_func == NULL || !system_initialized) {
        return _FALSE;
    }
    
    if (ms_time == 0) {
        ms_time = TIMER_TICK_INTERVAL_MS;
    }
    
    // 计算需要的tick数
    timer_ticks = (ms_time + TIMER_TICK_INTERVAL_MS - 1) / TIMER_TICK_INTERVAL_MS;
    if (timer_ticks == 0) {
        timer_ticks = 1;
    }
    
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
    
    // 首先检查是否已存在相同的事件
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        if (timer_events[i].is_active && 
            timer_events[i].event_callback == event_func) {
            // 更新现有事件
            timer_events[i].time_remaining = timer_ticks;
            timer_events[i].interval_ms = ms_time;
            result = _TRUE;
            goto exit;
        }
    }
    
    // 查找空闲槽位
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        if (!timer_events[i].is_active) {
            timer_events[i].event_callback = event_func;
            timer_events[i].time_remaining = timer_ticks;
            timer_events[i].interval_ms = ms_time;
            timer_events[i].is_active = 1;
            result = _TRUE;
            goto exit;
        }
    }
    
exit:
    rtos_mutex_give(timer_event_mutex);
    return result;
}

/*============================================================================*/
/**
 * 取消特定的定时器事件
 */
void fw_timer_event_cancel(void (*event_func)(void))
{
    uint8_t i;
    
    if (event_func == NULL || !system_initialized) {
        return;
    }
    
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
    
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        if (timer_events[i].is_active && 
            timer_events[i].event_callback == event_func) {
            
            // 清除事件信息
            timer_events[i].event_callback = NULL;
            timer_events[i].time_remaining = 0;
            timer_events[i].interval_ms = 0;
            timer_events[i].is_active = 0;
        }
    }
    
    rtos_mutex_give(timer_event_mutex);
}

/*============================================================================*/
/**
 * 取消所有定时器事件
 */
void fw_timer_event_cancel_all(void)
{
    uint8_t i;
    
    if (!system_initialized) {
        return;
    }
    
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
    
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        timer_events[i].event_callback = NULL;
        timer_events[i].time_remaining = 0;
        timer_events[i].interval_ms = 0;
        timer_events[i].is_active = 0;
    }
    
    rtos_mutex_give(timer_event_mutex);
}

/*============================================================================*/
/**
 * 获取系统状态信息（调试用）
 */
void fw_timer_event_status(void)
{
    uint8_t i;
    uint8_t active_count = 0;
    
    if (!system_initialized) {
        printf("Timer event system not initialized\n");
        return;
    }
    
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
    
    printf("=== Timer Event System Status ===\n");
    for (i = 0; i < MAX_TIMER_EVENTS; i++) {
        if (timer_events[i].is_active) {
            printf("Event[%d]: callback=0x%p, remaining=%d, interval=%d\n", 
                   i, timer_events[i].event_callback, 
                   timer_events[i].time_remaining, 
                   timer_events[i].interval_ms);
            active_count++;
        }
    }
    printf("Active events: %d/%d\n", active_count, MAX_TIMER_EVENTS);
    
    rtos_mutex_give(timer_event_mutex);
}

/*============================================================================*/
/**
 * 使用示例
 */
void example_event_1(void)
{
    printf("Event 1 triggered! (every 1000ms)\n");
}

void example_event_2(void)
{
    printf("Event 2 triggered! (every 500ms)\n");
}

void timer_event_example_task(void *param)
{
    (void)param; // 避免未使用参数警告
    
    // 初始化定时器事件系统
    fw_timer_event_init();
    
    // 等待系统初始化完成
    rtos_time_delay_ms(100);
    
    // 激活事件：1000ms周期调用example_event_1
    if (fw_timer_event_active(1000, example_event_1)) {
        printf("Event 1 activated successfully\n");
    } else {
        printf("Failed to activate event 1\n");
    }
    
    // 激活事件：500ms周期调用example_event_2
    if (fw_timer_event_active(500, example_event_2)) {
        printf("Event 2 activated successfully\n");
    } else {
        printf("Failed to activate event 2\n");
    }
    
    // 显示状态
    rtos_time_delay_ms(100);
    fw_timer_event_status();
    
    // 主任务可以做其他工作
    while (1) {
        // 其他业务逻辑
        rtos_time_delay_ms(1000);
        
        // 可以动态管理事件
        // fw_timer_event_cancel(example_event_1);  // 取消事件1
        // fw_timer_event_active(2000, some_other_event);  // 添加新事件
    }
}

/*============================================================================*/
/**
 * 启动定时器事件系统
 */
void start_timer_event_system(void)
{
    rtos_task_t task_handle;
    
    // 创建定时器事件示例任务
    if (rtos_task_create(&task_handle,
                        "TimerEventExample",
                        timer_event_example_task,
                        NULL,
                        TIMER_EVENT_STACK_SIZE * 4, // 字节为单位
                        TIMER_EVENT_TASK_PRIORITY) != RTK_SUCCESS) {
        printf("Failed to create timer event task!\n");
    } else {
        printf("Timer event task created successfully\n");
    }
}

/*============================================================================*/
/**
 * @brief  Application example function
 * @param  None
 * @return None
 */
void app_example(void)
{
    printf("hello world\n");
    printf("Starting stepper motor initialization...\n");
        
    stepper_motor_init();
    
    printf("Stepper motor initialized successfully!\n");

    stepper_motor_set_direction(MOTOR_BASE, Motor_Direction_Forward, 900);
    printf("Setting base motor to forward direction at 900 steps/s...\n");
    
        // 2. 初始化任务管理系统
    task_manager_init();
    
    // 3. 启动所有应用任务
    task_manager_start_all();

   // DisplayLCD_Init();
   
}