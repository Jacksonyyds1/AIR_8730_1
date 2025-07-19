#include "ameba_soc.h"
#include "device.h"
#include "gpio_api.h"
#include "gpio_irq_api.h"
#include "pwmout_api.h"
#include "timer_api.h"
#include "us_ticker_api.h"
#include "os_wrapper.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Fan_motor.h"

// PID控制器结构体
typedef struct {
    float kp, ki, kd;                 // PID参数
    int32_t max_output, min_output;   // 输出限制
    float err_integral;              // 积分累积
    int32_t last_error;             //上次误差
} pid_control_block_t;

// FG信号运行时数据
typedef struct {
    uint32_t pulse_cnt;      //脉冲计数
    uint32_t start_time_us; //开始时间
    volatile bool isr_lock; //中断锁
    bool is_first_edge;     //首次边缘标志
    uint32_t last_pulse_interval; //上一个脉冲间隔
    uint32_t total_accumulated_time; //12个脉冲总累计时间
} fan_fg_runtime_t;

// 风扇速度运行时数据
typedef struct {
    bool auto_mode;
    int real_speed;
    int speed;
    int auto_speed;
    bool te_mode_active;
    int te_rpm;
    int realtime_rpm;
    int filtered_rpm;
    int target_rpm;
    int current_duty;
    pid_control_block_t pcb; //PID控制块
} fan_speed_runtime_t;

// 全局变量
static const int *rpm_table = NULL;
static int table_size = 0;
static fan_fg_runtime_t fg;
static fan_speed_runtime_t fan_speed_runtime;

// RTOS相关
static rtos_task_t fan_speed_task_handle = NULL;
static rtos_sema_t fan_speed_sema = NULL;

// 硬件接口
static pwmout_t fan_pwm;
static gpio_irq_t fan_fg_irq;
static gtimer_t timeout_timer;

//================================ PID控制器实现 ================================

void pid_control_block_init(pid_control_block_t *pcb, float kp, float ki, float kd, 
                           int32_t max_output, int32_t min_output)
{
    pcb->kp = kp;
    pcb->ki = ki;
    pcb->kd = kd;
    pcb->max_output = max_output;
    pcb->min_output = min_output;
    pcb->err_integral = 0;
    pcb->last_error = 0;
}

void pid_reset(pid_control_block_t *pcb)
{
    pcb->err_integral = 0;
    pcb->last_error = 0;
}

int32_t pid_iterate(pid_control_block_t *pcb, int32_t target_value, int32_t feedback_value)
{
    int32_t output = 0;
    int32_t error = target_value - feedback_value;
    int32_t err_delta = error - pcb->last_error; //本次误差和上次误差的差值(微分项)
    pcb->err_integral += error;                  //累积误差(积分项)
    
    // PID计算
    output = (int32_t)(pcb->kp * error + pcb->ki * pcb->err_integral + pcb->kd * err_delta);
    pcb->last_error = error;
    
    // 限制输出范围，防止积分饱和
    if(output >= pcb->max_output) {
        pcb->err_integral -= error; //撤销本次积分累积
        output = pcb->max_output;
    } else if(output <= pcb->min_output) {
        pcb->err_integral -= error; //撤销本次积分累积
        output = pcb->min_output;
    }
    
    return output;
}

//================================ 中位数滤波器 ================================

static int compare_int(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

int fan_rpm_median_filter(int rpm)
{
    #define FILTER_SIZE 9
    static int rpm_buffer[FILTER_SIZE];
    static int rpm_index = 0;

    rpm_buffer[rpm_index] = rpm;
    rpm_index = (rpm_index + 1) % FILTER_SIZE;

    int sorted_buffer[FILTER_SIZE];
    memcpy(sorted_buffer, rpm_buffer, sizeof(rpm_buffer));
    qsort(sorted_buffer, FILTER_SIZE, sizeof(int), compare_int);

    if(FILTER_SIZE % 2 == 0) {
        return (sorted_buffer[FILTER_SIZE / 2] + sorted_buffer[FILTER_SIZE / 2 - 1]) / 2;
    } else {
        return sorted_buffer[FILTER_SIZE / 2];  
    }
}

//================================ 超时定时器处理 ================================

void timeout_timer_handler(uint32_t id)
{
    UNUSED(id);
    // 100ms超时，表示FG信号丢失
    if(!fg.isr_lock) {
        fg.is_first_edge = true;
        fg.pulse_cnt = 0;
        fan_speed_runtime.realtime_rpm = 0;
        
        // 通知任务处理
        if(fan_speed_sema != NULL) {
            rtos_sema_give(fan_speed_sema);
        }
    }
}

//================================ FG信号中断处理 ================================

void fan_fg_isr_handler(uint32_t id, gpio_irq_event event)
{
    UNUSED(id);
    UNUSED(event);
    uint32_t current_time = us_ticker_read();
    
    if(fg.isr_lock) {
        return; // 主循环计算时忽略FG中断
    }
    
    if(fg.is_first_edge) {
        // 忽略第一次边缘，重置参数
        fg.is_first_edge = false;
        fg.start_time_us = current_time;
        fg.last_pulse_interval = 0xFFFFFFFF;
        fg.total_accumulated_time = 0;
        fg.pulse_cnt = 0;
        
        // 重启超时定时器
        gtimer_stop(&timeout_timer);
        gtimer_start_one_shout(&timeout_timer, 100000, timeout_timer_handler, NULL);
    } else {
        uint32_t pulse_interval = current_time - fg.start_time_us;
        
        // 忽略小于半个上次脉冲宽度的边缘(抗干扰)
        if(fg.last_pulse_interval == 0xFFFFFFFF || 
           pulse_interval > fg.last_pulse_interval / 2) {
            
            fg.pulse_cnt++;
            fg.total_accumulated_time += pulse_interval; //累计每个间隔
            fg.last_pulse_interval = pulse_interval;
            fg.start_time_us = current_time;
            
            // 重启超时定时器
            gtimer_stop(&timeout_timer);
            gtimer_start_one_shout(&timeout_timer, 100000, timeout_timer_handler, NULL);
            
            // 每圈计算一次转速
            if(fg.pulse_cnt == FAN_FG_PULSE_PER_CYCLE) {
                if(fan_speed_sema != NULL) {
                    rtos_sema_give(fan_speed_sema);
                }
            }
        }
    }
}

//================================ 风扇速度控制任务 ================================

void fan_speed_control_task(void *param)
{
    UNUSED(param);
    while(1) {
        // 等待信号量，超时时间100ms
        rtos_sema_take(fan_speed_sema, 100);
        
        // 锁定FG中断，避免在计算过程中被打断
        fg.isr_lock = true;
        uint32_t pulse_cnt = fg.pulse_cnt;
        uint32_t total_time_us = 0;
        
        if(pulse_cnt > 0) {
            total_time_us = fg.total_accumulated_time;
        }
        
        // 重置计数器
        fg.pulse_cnt = 0;
        fg.total_accumulated_time = 0;
        fg.isr_lock = false;
        
        // 更新速度档位
        if(fan_speed_runtime.auto_mode) {
            fan_speed_runtime.real_speed = fan_speed_runtime.auto_speed;
        } else {
            fan_speed_runtime.real_speed = fan_speed_runtime.speed;
        }
        
        // TE模式处理
        if(fan_speed_runtime.te_mode_active) {
            fan_speed_runtime.target_rpm = fan_speed_runtime.te_rpm;
        } else {
            if(fan_speed_runtime.real_speed < table_size) {
                fan_speed_runtime.target_rpm = rpm_table[fan_speed_runtime.real_speed];
            }
        }
        
        // 计算转速
        if(total_time_us == 0 || pulse_cnt == 0) {
            fan_speed_runtime.realtime_rpm = 0;
        } else {
            // RPM = (脉冲数 / 每转脉冲数) * (60 * 1000000微秒/分钟) / 总时间微秒
            fan_speed_runtime.realtime_rpm = 
                (pulse_cnt * 60 * 1000000) / (FAN_FG_PULSE_PER_CYCLE * total_time_us);
        }
        
        // 滤波处理
        fan_speed_runtime.filtered_rpm = fan_rpm_median_filter(fan_speed_runtime.realtime_rpm);
        
        // PID控制计算
        int32_t pid_output = pid_iterate(&fan_speed_runtime.pcb, 
                                        fan_speed_runtime.target_rpm, 
                                        fan_speed_runtime.filtered_rpm);
        
        fan_speed_runtime.current_duty = PID_OUTPUT_BIAS + pid_output;
        
        // 限制PWM占空比范围
        if(fan_speed_runtime.current_duty < 0) {
            fan_speed_runtime.current_duty = 0;
        } else if(fan_speed_runtime.current_duty > FAN_PWM_MAX_DUTY) {
            fan_speed_runtime.current_duty = FAN_PWM_MAX_DUTY;
        }
        
        // 设置PWM输出
        float duty_ratio = (float)fan_speed_runtime.current_duty / FAN_PWM_MAX_DUTY;
        pwmout_write(&fan_pwm, duty_ratio);
        
        printf("Target: %d RPM, Current: %d RPM, Duty: %d\r\n", 
               fan_speed_runtime.target_rpm, 
               fan_speed_runtime.filtered_rpm, 
               fan_speed_runtime.current_duty);
    }
}

//================================ 外部接口函数 ================================

void fan_speed_te_set(int rpm)
{
    fan_speed_runtime.te_mode_active = true;
    fan_speed_runtime.te_rpm = rpm;
}

void fan_speed_set_auto_mode(bool enable)
{
    fan_speed_runtime.auto_mode = enable;
}

void fan_speed_set_speed(int speed, bool auto_mode)
{
    if(speed < 0 || speed >= table_size) {
        printf("Invalid speed index: %d\r\n", speed);
        return;
    }

    if(auto_mode) {
        fan_speed_runtime.auto_speed = speed;
    } else {
        fan_speed_runtime.speed = speed;
    }
}

int fan_speed_get_real_speed(void)
{
    return fan_speed_runtime.real_speed;
}

int fan_speed_get_rpm(void)
{
    return fan_speed_runtime.filtered_rpm;
}

int fan_speed_get_fg_pps(void)
{
    return fan_speed_runtime.realtime_rpm * FAN_FG_PULSE_PER_CYCLE / 60;
}

int fan_speed_get_current_duty(void)
{
    return fan_speed_runtime.current_duty;
}

//================================ 初始化函数 ================================

void fan_speed_controller_init(const int *table, int size)
{
    if(table == NULL || size <= 0) {
        printf("Invalid RPM table\r\n");
        return;
    }

    rpm_table = table;
    table_size = size;

    // 初始化运行时参数
    memset(&fan_speed_runtime, 0, sizeof(fan_speed_runtime));
    fan_speed_runtime.speed = 1;
    fan_speed_runtime.auto_speed = 1;
    
    memset(&fg, 0, sizeof(fg));
    fg.is_first_edge = true;
    fg.last_pulse_interval = 0xFFFFFFFF;

    // 初始化PWM输出
    fan_pwm.pwm_idx = 0;  // 使用Channel 0
    fan_pwm.period = 0;
    pwmout_init(&fan_pwm, FAN_PWM_PIN);
    pwmout_period_us(&fan_pwm, 1000000 / FAN_PWM_FREQ); // 设置PWM周期
    pwmout_write(&fan_pwm, 0.0); // 初始占空比为0

    // 初始化FG信号捕获
    gpio_irq_init(&fan_fg_irq, FAN_FG_PIN, fan_fg_isr_handler, 0);
    gpio_irq_set(&fan_fg_irq, IRQ_RISE, 1); // 上升沿触发
    gpio_irq_pull_ctrl(&fan_fg_irq, PullUp);
    gpio_irq_enable(&fan_fg_irq);

    // 初始化超时定时器
    gtimer_init(&timeout_timer, TIMER4);

    // 初始化PID控制器
    pid_control_block_init(&fan_speed_runtime.pcb, PID_KP, PID_KI, PID_KD, 
                          PID_OUTPUT_MAX, PID_OUTPUT_MIN);

    // 创建信号量
    if(rtos_sema_create_binary(&fan_speed_sema) != RTK_SUCCESS) {
        printf("Failed to create fan speed semaphore\r\n");
        return;
    }

    // 创建控制任务
    if(rtos_task_create(&fan_speed_task_handle, "FanSpeedCtrl", 
                       fan_speed_control_task, NULL, 2048, 4) != RTK_SUCCESS) {
        printf("Failed to create fan speed control task\r\n");
    } else {
        printf("Fan speed control task created successfully\r\n");
    }
}

//================================ 使用示例 ================================

// RPM表示例
static const int fan_rpm_table[] = {
    0,     // 档位0: 停转
    500,  // 档位1: 1000 RPM
    1500,  // 档位2: 1500 RPM
    2000,  // 档位3: 2000 RPM
    2500,  // 档位4: 2500 RPM
    3000,  // 档位5: 3000 RPM
};

void fan_controller_example(void)
{
    // 初始化风扇控制器
    fan_speed_controller_init(fan_rpm_table, sizeof(fan_rpm_table)/sizeof(fan_rpm_table[0]));

    // 设置为手动模式，档位2
    fan_speed_set_auto_mode(false);
    fan_speed_set_speed(3, false);
    
    // 或者设置TE模式，直接指定转速
    // fan_speed_te_set(1800);
    
    printf("Fan controller initialized\r\n");
}

void test_pwm_output(void)
{
    pwmout_t test_pwm;
    test_pwm.pwm_idx = 3;
    test_pwm.period = 0;
    
    pwmout_init(&test_pwm, _PB_30);
    pwmout_period_us(&test_pwm, 250); // 1ms周期
    pwmout_write(&test_pwm, 0.5);      // 50%占空比
    
    printf("PWM test output initialized\r\n");
}