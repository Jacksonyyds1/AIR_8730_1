#ifndef TIMER_EVENT_H
#define TIMER_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ameba_soc.h"  

#define  _FALSE 0
#define  _TRUE  1
void fw_timer_event_Init(void);
extern void fw_timer_event_CancelTimerEvent(void (*Event)(void));
extern void fw_timer_event_CancelAllTimerEvent(void);
extern uint8_t fw_timer_event_ActiveTimerEvent(uint16_t msTime, void (*Event)(void));
extern void fw_timer_event_Handler(void);
extern void fw_timer_event_isr_1ms(void);

#ifdef __cplusplus
}
#endif

#endif  // TIMER_EVENT_H