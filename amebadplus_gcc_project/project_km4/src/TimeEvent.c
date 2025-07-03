#include "ameba_soc.h"     // RTL8721D主头文件
#include "TimeEvent.h"     // 自定义头文件
#include "os_wrapper.h"    // RTOS相关

/*===================================  Types ================================*/
typedef struct TIMEREVENT_STRUCT
{
    uint16_t Time;              ///! The rest time of this event
    uint16_t SetTimeInterval;   ///! The rest time of this event
    void (*Event)(void);        ///! The event handler's function 
} _TIMEREVENT_STRUCT;

/*================================ Definitions ==============================*/
#define _INACTIVE_TIMER_EVENT   (0xFFFF)
#define _MAX_EVENT_AMOUNT       (12)    ///! The amount of all soft timer events.

/*================================== Variables ==============================*/
static struct TIMEREVENT_STRUCT TimerEvent[_MAX_EVENT_AMOUNT];

// 添加互斥锁保护（可选，用于多任务环境）
#ifdef CONFIG_RTOS
static rtos_mutex_t timer_event_mutex = NULL;
#endif

/*============================================================================*/
/**
 * Initialize timer event system
 */
void fw_timer_event_Init(void)
{
    uint8_t i;
    
    // 初始化定时器事件数组
    for (i = 0; i < _MAX_EVENT_AMOUNT; i++)
    {
        TimerEvent[i].Time = _INACTIVE_TIMER_EVENT;
        TimerEvent[i].SetTimeInterval = 0;
        TimerEvent[i].Event = NULL;
    }
    
#ifdef CONFIG_RTOS
    // 创建互斥锁
    rtos_mutex_create(&timer_event_mutex);
#endif
}

/*============================================================================*/
/**
 * Deinitialize timer event system
 */
void fw_timer_event_Deinit(void)
{
#ifdef CONFIG_RTOS
    if (timer_event_mutex != NULL) {
        rtos_mutex_delete(timer_event_mutex);
        timer_event_mutex = NULL;
    }
#endif
}

/*============================================================================*/
/**
 * Cancel a timer event
 */
void fw_timer_event_CancelTimerEvent(void (*Event)(void))
{
    uint8_t timereventcnt;

#ifdef CONFIG_RTOS
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
#endif

    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Event == Event)
        {
            TimerEvent[timereventcnt].Time = _INACTIVE_TIMER_EVENT;
            TimerEvent[timereventcnt].Event = NULL;
        }
    }

#ifdef CONFIG_RTOS
    rtos_mutex_give(timer_event_mutex);
#endif
}

/*============================================================================*/
/**
 * Cancel all timer events
 */
void fw_timer_event_CancelAllTimerEvent(void)
{
    uint8_t timereventcnt;

#ifdef CONFIG_RTOS
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
#endif

    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        TimerEvent[timereventcnt].Time = _INACTIVE_TIMER_EVENT;
        TimerEvent[timereventcnt].SetTimeInterval = 0;
        TimerEvent[timereventcnt].Event = NULL;
    }

#ifdef CONFIG_RTOS
    rtos_mutex_give(timer_event_mutex);
#endif
}

/*============================================================================*/
/**
 * Add new function into timer event
 */
uint8_t fw_timer_event_ActiveTimerEvent(uint16_t msTime, void (*Event)(void))
{
    uint8_t timereventcnt;
    uint16_t stTimeInterval;
    uint8_t result = _FALSE;

    if (Event == NULL) {
        return _FALSE;
    }

    stTimeInterval = msTime;
    if (!stTimeInterval) {
        stTimeInterval = 1;
    }

#ifdef CONFIG_RTOS
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
#endif

    // 检查是否已经存在相同的事件
    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if ((TimerEvent[timereventcnt].Time != _INACTIVE_TIMER_EVENT) && 
            (TimerEvent[timereventcnt].Event == Event))
        {
            TimerEvent[timereventcnt].Time = stTimeInterval;
            TimerEvent[timereventcnt].SetTimeInterval = stTimeInterval;
            result = _FALSE;
            goto exit;
        }
    }

    // 查找空闲的槽位
    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Time == _INACTIVE_TIMER_EVENT)
        {
            TimerEvent[timereventcnt].Time = stTimeInterval;
            TimerEvent[timereventcnt].SetTimeInterval = stTimeInterval;
            TimerEvent[timereventcnt].Event = Event;
            result = _TRUE;
            goto exit;
        }
    }

exit:
#ifdef CONFIG_RTOS
    rtos_mutex_give(timer_event_mutex);
#endif
    return result;
}

/*============================================================================*/
/**
 * Timer event handler - 在主循环中调用
 */
void fw_timer_event_Handler(void)
{
    uint8_t timereventcnt;

#ifdef CONFIG_RTOS
    rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
#endif

    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Time == 0)
        {
            TimerEvent[timereventcnt].Time = TimerEvent[timereventcnt].SetTimeInterval;
            
            // 调用回调函数
            if (TimerEvent[timereventcnt].Event != NULL)
            {
#ifdef CONFIG_RTOS
                rtos_mutex_give(timer_event_mutex);
#endif
                (*TimerEvent[timereventcnt].Event)();
#ifdef CONFIG_RTOS
                rtos_mutex_take(timer_event_mutex, RTOS_MAX_DELAY);
#endif
            }
        }
    }

#ifdef CONFIG_RTOS
    rtos_mutex_give(timer_event_mutex);
#endif
}

/*============================================================================*/
/**
 * 1ms中断服务函数中调用
 */
void fw_timer_event_isr_1ms(void)
{
    uint8_t timereventcnt;
    
    // 注意：在中断中不使用互斥锁，避免死锁
    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Time != _INACTIVE_TIMER_EVENT)
        {
            if (TimerEvent[timereventcnt].Time > 0)
            {
                TimerEvent[timereventcnt].Time--;
            }
        }
    }
}