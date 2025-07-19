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

#define FAN_PWM_TIMER_IDX         9 // 使用Timer 9作为PWM定时器
#define FAN_FG_CAPTURE_TIMER  8 // 使用Timer 8作为FG捕获定时器
#define FAN_TIMEOUT_TIMER        4 // 使用Timer 4作为超时定时器

/* PWM配置 - Timer9 */
#define PWM9_PRESCALER         39       // 与原有配置保持一致
#define PWM9_PERIOD            (1000000 / FAN_PWM_FREQ)  // 根据频率计算周期
#define PWM9_CHANNEL           0        // 使用Timer9的Channel 0

/* FG捕获配置 - Timer8 */
#define FG_CAPTURE_CHANNEL     0        // 使用Timer8的Channel 0
#define FG_CAPTURE_MODE        TIM_CCMode_PulseWidth  // 脉宽测量模式

#define FAN_PWM_PIN_TIMER9     _PB_30   // Timer9 PWM输出引脚
#define FAN_FG_PIN_TIMER8      _PB_31   // Timer8捕获输入引脚
// PID控制器结构体
typedef struct {
    float kp, ki, kd;                 // PID参数
    int32_t max_output, min_output;   // 输出限制
    float err_integral;              // 积分累积
    int32_t last_error;             //上次误差
} pid_control_block_t;

typedef struct {
    uint32_t pulse_count;
    uint32_t pulse_width_us;
    uint32_t last_capture_time;
    bool signal_valid;
    uint32_t pulse_interval_us;
} fan_fg_capture_data_t;

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
static fan_fg_capture_data_t fg_capture;
static fan_speed_runtime_t fan_speed_runtime;

// RTOS相关
static rtos_task_t fan_speed_task_handle = NULL;
static rtos_sema_t fan_speed_sema = NULL;
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
/* 
void timeout_timer_handler(uint32_t id)
{
    UNUSED(id);
    // 100ms超时，表示FG信号丢失
    if(!fg_capture.isr_lock) {
        fg_capture.is_first_edge = true;
        fg_capture.pulse_count = 0;
        fan_speed_runtime.realtime_rpm = 0;
        
        // 通知任务处理
        if(fan_speed_sema != NULL) {
            rtos_sema_give(fan_speed_sema);
        }
    }
} */

//================================ Timer9 PWM实现 ================================
/**
 * @brief 初始化Timer9 PWM输出
 */
void timer9_pwm_init(void)
{
    RTIM_TimeBaseInitTypeDef RTIM_InitStruct;
    TIM_CCInitTypeDef TIM_CCInitStruct;

    /* 使能Timer9时钟 */
    RCC_PeriphClockCmd(APBPeriph_TIMx[FAN_PWM_TIMER_IDX], 
                       APBPeriph_TIMx_CLOCK[FAN_PWM_TIMER_IDX], ENABLE);

    /* Timer9基础配置 */
    RTIM_TimeBaseStructInit(&RTIM_InitStruct);
    RTIM_InitStruct.TIM_Idx = FAN_PWM_TIMER_IDX;
    RTIM_InitStruct.TIM_Prescaler = PWM9_PRESCALER;
    RTIM_InitStruct.TIM_Period = PWM9_PERIOD - 1;
    RTIM_TimeBaseInit(TIMx[FAN_PWM_TIMER_IDX], &RTIM_InitStruct, 
                      TIMx_irq[FAN_PWM_TIMER_IDX], NULL, NULL);

    /* PWM通道配置 */
    RTIM_CCStructInit(&TIM_CCInitStruct);
    TIM_CCInitStruct.TIM_OCPulse = 0;  // 初始占空比为0
    RTIM_CCxInit(TIMx[FAN_PWM_TIMER_IDX], &TIM_CCInitStruct, PWM9_CHANNEL);
    RTIM_CCxCmd(TIMx[FAN_PWM_TIMER_IDX], PWM9_CHANNEL, TIM_CCx_Enable);

    /* 配置引脚复用 */
    Pinmux_Config(FAN_PWM_PIN_TIMER9, PINMUX_FUNCTION_PWM0);

    /* 启动Timer9 */
    RTIM_Cmd(TIMx[FAN_PWM_TIMER_IDX], ENABLE);
    
    printf("Timer9 PWM initialized for fan control\n");
}

/**
 * @brief 设置Timer9 PWM占空比
 * @param duty_percent: 占空比百分比 (0.0-1.0)
 */
void timer9_pwm_set_duty(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 1.0f) duty_percent = 1.0f;
    
    uint32_t pulse_width = (uint32_t)(PWM9_PERIOD * duty_percent);
    RTIM_CCRxSet(TIMx[FAN_PWM_TIMER_IDX], pulse_width, PWM9_CHANNEL);
}
//================================ Timer8 FG信号中断处理 ================================

/**
 * @brief Timer8 FG信号捕获中断处理函数
 */
static u32 timer8_fg_capture_isr(void *data)
{
    UNUSED(data);
    
    static uint32_t last_capture = 0;
    uint32_t current_capture = RTIM_CCRxGet(TIMx[FAN_FG_CAPTURE_TIMER], FG_CAPTURE_CHANNEL);
    
    fg_capture.pulse_count++;
    
    if (last_capture != 0) {
        if (current_capture > last_capture) {
            fg_capture.pulse_interval_us = current_capture - last_capture;
        } else {
            // 处理定时器溢出情况
            fg_capture.pulse_interval_us = (0xFFFF - last_capture) + current_capture;
        }
        fg_capture.signal_valid = true;
    }
    
    last_capture = current_capture;
    fg_capture.last_capture_time = us_ticker_read();
    
    // 每12个脉冲(一圈)通知主任务
    if (fg_capture.pulse_count >= FAN_FG_PULSE_PER_CYCLE) {
        if (fan_speed_sema != NULL) {
            rtos_sema_give(fan_speed_sema);
        }
    }
    
    RTIM_INTClear(TIMx[FAN_FG_CAPTURE_TIMER]);
    return 0;
}

/**
 * @brief 初始化Timer8 FG信号捕获
 */
static void timer8_fg_capture_init(void)
{
    RTIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_CCInitTypeDef TIM_CCInitStruct;

    /* 使能Timer8时钟 */
    RCC_PeriphClockCmd(APBPeriph_TIMx[FAN_FG_CAPTURE_TIMER], 
                       APBPeriph_TIMx_CLOCK[FAN_FG_CAPTURE_TIMER], ENABLE);

    /* Timer8基础配置 */
    RTIM_TimeBaseStructInit(&TIM_InitStruct);
    TIM_InitStruct.TIM_Idx = FAN_FG_CAPTURE_TIMER;
    RTIM_TimeBaseInit(TIMx[FAN_FG_CAPTURE_TIMER], &TIM_InitStruct, 
                      TIMx_irq[FAN_FG_CAPTURE_TIMER], 
                      (IRQ_FUN)timer8_fg_capture_isr, 0);

    /* 配置输入捕获 */
    RTIM_CCStructInit(&TIM_CCInitStruct);
    TIM_CCInitStruct.TIM_ICPulseMode = TIM_CCMode_Inputcapture;  // 输入捕获模式
    RTIM_CCxInit(TIMx[FAN_FG_CAPTURE_TIMER], &TIM_CCInitStruct, FG_CAPTURE_CHANNEL);
    RTIM_INTConfig(TIMx[FAN_FG_CAPTURE_TIMER], TIM_IT_CC0, ENABLE);
    RTIM_CCxCmd(TIMx[FAN_FG_CAPTURE_TIMER], FG_CAPTURE_CHANNEL, TIM_CCx_Enable);

    /* 配置引脚复用为定时器输入 */
    Pinmux_Config(FAN_FG_PIN_TIMER8, PINMUX_FUNCTION_TIMER);
    PAD_PullCtrl(FAN_FG_PIN_TIMER8, GPIO_PuPd_UP);

    /* 启动Timer8 */
    RTIM_Cmd(TIMx[FAN_FG_CAPTURE_TIMER], ENABLE);
    
    printf("Timer8 FG capture initialized\n");
}

//================================   速度计算优化   ================================
/**
 * @brief 计算风扇转速（基于硬件捕获数据）
 * @return 当前转速(RPM)
 */
static int calculate_fan_rpm_from_capture(void)
{
    if (!fg_capture.signal_valid || fg_capture.pulse_interval_us == 0) {
        return 0;
    }
    
    // RPM = (60 * 1000000) / (脉冲间隔微秒 * 每转脉冲数)
    int rpm = (60 * 1000000) / (fg_capture.pulse_interval_us * FAN_FG_PULSE_PER_CYCLE);
    
    return rpm;
}
//================================ 风扇速度控制任务 ================================

void fan_speed_control_task(void *param)
{
    UNUSED(param);
    
    while(1) {
        // 等待信号量，超时时间200ms
        if (rtos_sema_take(fan_speed_sema, 200) == RTK_SUCCESS) {
            // 获取硬件捕获的转速数据
            fan_speed_runtime.realtime_rpm = calculate_fan_rpm_from_capture();
            
            // 重置脉冲计数
            fg_capture.pulse_count = 0;
        } else {
            // 超时，可能风扇停转
            fan_speed_runtime.realtime_rpm = 0;
            fg_capture.signal_valid = false;
        }
        
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
        
        // 使用Timer9设置PWM输出
        float duty_ratio = (float)fan_speed_runtime.current_duty / FAN_PWM_MAX_DUTY;
        timer9_pwm_set_duty(duty_ratio);
        
        printf("Target: %d RPM, Current: %d RPM, Duty: %.2f%%, Interval: %lu us\r\n", 
               fan_speed_runtime.target_rpm, 
               fan_speed_runtime.filtered_rpm,
               duty_ratio * 100,
               fg_capture.pulse_interval_us);
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
    memset(&fg_capture, 0, sizeof(fg_capture));
    
    fan_speed_runtime.speed = 1;
    fan_speed_runtime.auto_speed = 1;

    // 初始化Timer9 PWM输出
    timer9_pwm_init();
    
    // 初始化Timer8 FG信号捕获
    timer8_fg_capture_init();

    // 初始化超时定时器(Timer4)
    gtimer_init(&timeout_timer, FAN_TIMEOUT_TIMER);

    // 初始化PID控制器
    pid_control_block_init(&fan_speed_runtime.pcb, PID_KP, PID_KI, PID_KD, 
                          PID_OUTPUT_MAX, PID_OUTPUT_MIN);

    // 创建信号量
    if(rtos_sema_create_binary(&fan_speed_sema) != RTK_SUCCESS) {
        printf("Failed to create fan speed semaphore\r\n");
        return;
    }

    // 创建改进后的控制任务
    if(rtos_task_create(&fan_speed_task_handle, "FanSpeedCtrlV2", 
                       fan_speed_control_task, NULL, 2048, 4) != RTK_SUCCESS) {
        printf("Failed to create fan speed control task\r\n");
    } else {
        printf("Improved fan speed control task created successfully\r\n");
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
    
    // 设置为手动模式，档位1
    fan_speed_set_auto_mode(false);
    fan_speed_set_speed(1, false);
    
    // 或者设置TE模式，直接指定转速
    // fan_speed_te_set(1800);
    
    printf("Fan controller initialized\r\n");
}

void test_pwm_output(void)
{
    timer9_pwm_init();
    timer9_pwm_set_duty(0.5);  // 50%占空比
    printf("PWM test output initialized\r\n");
}