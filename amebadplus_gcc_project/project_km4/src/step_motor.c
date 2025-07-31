#include "step_motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "platform_autoconf.h"

// --- 步进电机结构体定义 ---
typedef struct {
    Motor_State_t state;
    Motor_Running_Type_t running_type;
    
    bool motor_break;
    
    uint8_t accelerate_rate;
    uint16_t accelerate_step;
    Motor_Direction_t current_direction;
    Motor_Direction_t target_direction;

    int position;
    int target_position;

    uint16_t current_speed;
    uint16_t target_speed;
} stepper_motor_t;

static QueueHandle_t encoder_sample_queue = NULL;

// --- 全局变量 ---
static stepper_motor_t stepper_motor[MOTOR_COUNT] = {
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        false, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        0, 0
    }, // MOTOR_NECK
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        false, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        0, 0
    }, // MOTOR_BASE
};

// GPIO对象和RTIM配置结构体
static gpio_t motor_gpio[MOTOR_COUNT][4];  // 每个电机4个GPIO引脚
static RTIM_TimeBaseInitTypeDef motor_rtim_config[MOTOR_COUNT];  // 每个电机的RTIM配置

// 单极性步进电机步序 (A+ B+ A- B-)
static const uint8_t unipolar_step_sequence[8] = {
    0b1000,  // Step 0
    0b1100,  // Step 1
    0b0100,  // Step 2
    0b0110,  // Step 3
    0b0010,  // Step 4
    0b0011,  // Step 5
    0b0001,  // Step 6
    0b1001   // Step 7
};

// --- 静态函数声明 ---
static void stepper_motor_start_timer(uint8_t index);
static void stepper_motor_stop_timer(uint8_t index);
static void set_stepper_motor_output(uint8_t index, int position, bool motor_break);
static void stepper_motor_position_handler(stepper_motor_t *motor);
static void stepper_motor_direction_handler(stepper_motor_t *motor);
static void stepper_motor_driver(uint8_t index);
static uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate);
static void stepper_motor_update_timer_period(uint8_t index);

// 定时器回调函数
static uint32_t timer_neck_handler(void *id);
static uint32_t timer_base_handler(void *id);

// --- GPIO初始化 ---
void stepper_motor_gpio_init(void)
{
    // 先将引脚配置为GPIO功能，取消SPI功能
    printf("GPIO init: Configuring pins as GPIO...\n");

    // MOTOR_BASE 引脚
    Pinmux_Config(MOTOR_BASE_A_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_B_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_A_MINUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_B_MINUS, PINMUX_FUNCTION_GPIO);

    printf("GPIO init: Pins configured as GPIO\n");

    // MOTOR_BASE GPIO初始化
    gpio_init(&motor_gpio[MOTOR_BASE][0], MOTOR_BASE_A_PLUS);
    gpio_init(&motor_gpio[MOTOR_BASE][1], MOTOR_BASE_B_PLUS);
    gpio_init(&motor_gpio[MOTOR_BASE][2], MOTOR_BASE_A_MINUS);
    gpio_init(&motor_gpio[MOTOR_BASE][3], MOTOR_BASE_B_MINUS);
    
    for(int i = 0; i < 4; i++) {
        gpio_dir(&motor_gpio[MOTOR_BASE][i], PIN_OUTPUT);
        gpio_mode(&motor_gpio[MOTOR_BASE][i], PullNone);
        gpio_write(&motor_gpio[MOTOR_BASE][i], 0);
    }
    
    // 如果需要MOTOR_NECK，取消注释以下代码
    /*
    // MOTOR_NECK 引脚
    Pinmux_Config(MOTOR_NECK_A_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_B_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_A_MINUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_B_MINUS, PINMUX_FUNCTION_GPIO);

    // MOTOR_NECK GPIO初始化
    gpio_init(&motor_gpio[MOTOR_NECK][0], MOTOR_NECK_A_PLUS);
    gpio_init(&motor_gpio[MOTOR_NECK][1], MOTOR_NECK_B_PLUS);
    gpio_init(&motor_gpio[MOTOR_NECK][2], MOTOR_NECK_A_MINUS);
    gpio_init(&motor_gpio[MOTOR_NECK][3], MOTOR_NECK_B_MINUS);

    for(int i = 0; i < 4; i++) {
        gpio_dir(&motor_gpio[MOTOR_NECK][i], PIN_OUTPUT);
        gpio_mode(&motor_gpio[MOTOR_NECK][i], PullNone);
        gpio_write(&motor_gpio[MOTOR_NECK][i], 0);
    }
    */
    
    printf("GPIO init: Completed successfully\n");
}

// --- 定时器初始化 ---
void stepper_motor_timer_init(void)
{
    printf("Timer init: Enabling timer clocks...\n");
    
    // 启用定时器时钟
    RCC_PeriphClockCmd(APBPeriph_TIMx[MOTOR_TIMER_NECK], APBPeriph_TIMx_CLOCK[MOTOR_TIMER_NECK], ENABLE);
    RCC_PeriphClockCmd(APBPeriph_TIMx[MOTOR_TIMER_BASE], APBPeriph_TIMx_CLOCK[MOTOR_TIMER_BASE], ENABLE);
    
    // 初始化RTIM配置结构体但不启动定时器
    RTIM_TimeBaseStructInit(&motor_rtim_config[MOTOR_NECK]);
    motor_rtim_config[MOTOR_NECK].TIM_Idx = MOTOR_TIMER_NECK;
    
    RTIM_TimeBaseStructInit(&motor_rtim_config[MOTOR_BASE]);
    motor_rtim_config[MOTOR_BASE].TIM_Idx = MOTOR_TIMER_BASE;
    
    printf("Timer init: RTIM configurations initialized\n");
}

// --- 动态更新定时器周期 ---
static void stepper_motor_update_timer_period(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    // 修正的周期计算逻辑
    // current_speed范围: 0-1000
    // 映射到合理的步进频率范围: 10Hz - 2000Hz
    uint32_t period_us;
    
    if(stepper_motor[index].current_speed == 0) {
        // 速度为0时，使用最大周期（最低频率）
        period_us = 100000; // 100ms = 10Hz
    } else {
        // 改进的映射算法: 
        // speed 1-1000 映射到频率 20Hz-2000Hz
        // 使用对数或线性映射提供更好的控制精度
        
        float speed_normalized = (float)stepper_motor[index].current_speed / 1000.0f; // 0.001 - 1.0
        
        // 线性映射到频率: 20Hz + (2000-20) * speed_ratio = 20 + 1980 * speed_ratio
        float target_frequency = 20.0f + 1980.0f * speed_normalized;
        
        // 转换为周期(微秒)
        period_us = (uint32_t)(1000000.0f / target_frequency);
        
        // 安全范围检查
        if(period_us < 500) period_us = 500;     // 最大2000Hz
        if(period_us > 50000) period_us = 50000; // 最小20Hz
    }
    
    // 计算RTIM周期值 (32.768kHz时钟，周期 = 30.517μs)
    uint32_t rtim_period = (uint32_t)((float)period_us / 1000000.0f * 32768.0f) - 1;
    
    // 确保RTIM周期在有效范围内 (至少15个tick，避免过快)
    if(rtim_period < 15) rtim_period = 15;
    if(rtim_period > 3276) rtim_period = 3276; // 对应约100ms
    
    // 如果周期发生变化，重新配置定时器
    if(rtim_period != motor_rtim_config[index].TIM_Period) {
        motor_rtim_config[index].TIM_Period = rtim_period;
        
        // 停止定时器
        stepper_motor_stop_timer(index);
        
        // 重新启动定时器（如果电机未停止）
        if(stepper_motor[index].state != Motor_State_Stop || 
           stepper_motor[index].current_speed > 0) {
            stepper_motor_start_timer(index);
        }
    }
}

// --- 定时器启动函数 ---
static void stepper_motor_start_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    // 使用与update函数相同的周期计算逻辑
    uint32_t period_us;
    
    if(stepper_motor[index].current_speed == 0) {
        period_us = 100000; // 100ms = 10Hz
    } else {
        // 改进的映射算法: speed 1-1000 映射到频率 20Hz-2000Hz
        float speed_normalized = (float)stepper_motor[index].current_speed / 1000.0f;
        float target_frequency = 20.0f + 1980.0f * speed_normalized;
        period_us = (uint32_t)(1000000.0f / target_frequency);
        
        // 安全范围检查
        if(period_us < 500) period_us = 500;     // 最大2000Hz
        if(period_us > 50000) period_us = 50000; // 最小20Hz
    }
    
    // 计算RTIM周期值 (32.768kHz时钟)
    uint32_t rtim_period = (uint32_t)((float)period_us / 1000000.0f * 32768.0f) - 1;
    
    // 确保RTIM周期在有效范围内
    if(rtim_period < 15) rtim_period = 15;
    if(rtim_period > 3276) rtim_period = 3276;
    
    // 更新配置
    motor_rtim_config[index].TIM_Period = rtim_period;
    
    // 根据电机索引选择相应的定时器和回调函数
    if(index == MOTOR_NECK) {
        RTIM_TimeBaseInit(TIMx[MOTOR_TIMER_NECK], &motor_rtim_config[MOTOR_NECK], 
                         TIMx_irq[MOTOR_TIMER_NECK], timer_neck_handler, 
                         (u32)&motor_rtim_config[MOTOR_NECK]);
        RTIM_INTConfig(TIMx[MOTOR_TIMER_NECK], TIM_IT_Update, ENABLE);
        RTIM_Cmd(TIMx[MOTOR_TIMER_NECK], ENABLE);
    } else if(index == MOTOR_BASE) {
        RTIM_TimeBaseInit(TIMx[MOTOR_TIMER_BASE], &motor_rtim_config[MOTOR_BASE], 
                         TIMx_irq[MOTOR_TIMER_BASE], timer_base_handler, 
                         (u32)&motor_rtim_config[MOTOR_BASE]);
        RTIM_INTConfig(TIMx[MOTOR_TIMER_BASE], TIM_IT_Update, ENABLE);
        RTIM_Cmd(TIMx[MOTOR_TIMER_BASE], ENABLE);
    }
}

// --- 定时器停止函数 ---
static void stepper_motor_stop_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    if(index == MOTOR_NECK) {
        RTIM_Cmd(TIMx[MOTOR_TIMER_NECK], DISABLE);
        RTIM_INTConfig(TIMx[MOTOR_TIMER_NECK], TIM_IT_Update, DISABLE);
    } else if(index == MOTOR_BASE) {
        RTIM_Cmd(TIMx[MOTOR_TIMER_BASE], DISABLE);
        RTIM_INTConfig(TIMx[MOTOR_TIMER_BASE], TIM_IT_Update, DISABLE);
    }
}

// --- 设置步进电机输出 ---
static void set_stepper_motor_output(uint8_t index, int position, bool motor_break)
{
    if(index >= MOTOR_COUNT) return;
    
    uint8_t step_pattern = unipolar_step_sequence[position & 0x07];
    
    if(motor_break) {
        // 输出步进序列
        gpio_write(&motor_gpio[index][0], (step_pattern & 0x08) ? 1 : 0); // A+
        gpio_write(&motor_gpio[index][1], (step_pattern & 0x04) ? 1 : 0); // B+
        gpio_write(&motor_gpio[index][2], (step_pattern & 0x02) ? 1 : 0); // A-
        gpio_write(&motor_gpio[index][3], (step_pattern & 0x01) ? 1 : 0); // B-
    } else {
        // 释放电机（所有引脚输出0）
        for(int i = 0; i < 4; i++) {
            gpio_write(&motor_gpio[index][i], 0);
        }
    }
}

// --- 位置控制处理 ---
static void stepper_motor_position_handler(stepper_motor_t *motor)
{
    if(motor->target_position == motor->position) {
        motor->state = Motor_State_Stopping;
        motor->target_direction = Motor_Direction_Stop;
    } else if(motor->target_position > motor->position) {
        motor->target_direction = Motor_Direction_Forward;
        motor->state = Motor_State_Forward;
        motor->position++;
    } else {
        motor->target_direction = Motor_Direction_Backward;
        motor->state = Motor_State_Backward;
        motor->position--;
    }
}

// --- 方向控制处理 ---
static void stepper_motor_direction_handler(stepper_motor_t *motor)
{
    if(motor->target_direction == Motor_Direction_Stop) {
        motor->state = Motor_State_Stopping;

        if(motor->current_speed == 0){
            motor->state = Motor_State_Stop;
        }
    } else if(motor->target_direction == Motor_Direction_Forward) {
        motor->state = Motor_State_Forward;
        motor->position++;
    } else if(motor->target_direction == Motor_Direction_Backward) {
        motor->state = Motor_State_Backward;
        motor->position--;
    }
    
    motor->current_direction = motor->target_direction;

    // 速度控制
    uint16_t target_speed = (motor->target_direction == Motor_Direction_Stop) ? 0 : motor->target_speed;
    
    if (target_speed > motor->current_speed) {
        motor->accelerate_step++;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    } else if (target_speed < motor->current_speed) {
        if(motor->accelerate_step > 0) motor->accelerate_step--;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);

        if(motor->current_speed == 0 && motor->target_direction == Motor_Direction_Stop) {
            motor->state = Motor_State_Stop;
        }
    }
}

// --- 速度处理函数 ---
static uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate)
{
    if(current_speed == target_speed) {
        return current_speed;
    }
    
    if(current_speed < target_speed) {
        uint16_t speed_increment = accelerate_rate;
        if(current_speed + speed_increment > target_speed) {
            return target_speed;
        }
        return current_speed + speed_increment;
    } else {
        uint16_t speed_decrement = accelerate_rate;
        if(current_speed < speed_decrement) {
            return 0;
        }
        if(current_speed - speed_decrement < target_speed) {
            return target_speed;
        }
        return current_speed - speed_decrement;
    }
}

// --- 电机驱动函数 ---
static void stepper_motor_driver(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor_t *motor = &stepper_motor[index];

    if (motor->running_type == Motor_Running_Type_Positioning) {
        stepper_motor_position_handler(motor);
    } else {
        stepper_motor_direction_handler(motor);
    }

    if (motor->state == Motor_State_Stop) {
        set_stepper_motor_output(index, motor->position, motor->motor_break);
    } else {
        set_stepper_motor_output(index, motor->position, true);
    }
}

// --- 定时器回调函数 ---
static uint32_t timer_neck_handler(void *id)
{
    RTIM_TimeBaseInitTypeDef *rtim_config = (RTIM_TimeBaseInitTypeDef *)id;
    
    // 执行步进控制
    stepper_motor_driver(MOTOR_NECK);
    
    // 清除中断标志
    RTIM_INTClear(TIMx[rtim_config->TIM_Idx]);
    RTIM_INTClear(TIMx[rtim_config->TIM_Idx]); // 确保清除
    
    // 检查是否需要停止定时器
    if(stepper_motor[MOTOR_NECK].state == Motor_State_Stop &&
       stepper_motor[MOTOR_NECK].current_speed == 0) {
        stepper_motor_stop_timer(MOTOR_NECK);
    } else {
        // 动态调整定时器周期
        stepper_motor_update_timer_period(MOTOR_NECK);
    }
    
    return 0;
}

int i = 0;
static uint32_t timer_base_handler(void *id)
{
    RTIM_TimeBaseInitTypeDef *rtim_config = (RTIM_TimeBaseInitTypeDef *)id;
    
    // 执行步进控制
    stepper_motor_driver(MOTOR_BASE);
    
    if(i==500){
    printf("base motor timer run!\n");
    i = 0;
}
    i++;
    // 清除中断标志
    RTIM_INTClear(TIMx[rtim_config->TIM_Idx]);
    RTIM_INTClear(TIMx[rtim_config->TIM_Idx]); // 确保清除
    
    // 检查是否需要停止定时器
    if(stepper_motor[MOTOR_BASE].state == Motor_State_Stop &&
       stepper_motor[MOTOR_BASE].current_speed == 0) {
        stepper_motor_stop_timer(MOTOR_BASE);
    } else {
        // 动态调整定时器周期
        stepper_motor_update_timer_period(MOTOR_BASE);
    }
    
    return 0;
}

// --- 公共API函数实现 ---

void stepper_motor_init(void)
{
    // 初始化GPIO
    stepper_motor_gpio_init();
    
    // 初始化定时器
    stepper_motor_timer_init();
    
    printf("Stepper motor system initialized on RTL8721DCM with RTIM\n");
}

Motor_State_t stepper_motor_get_state(uint8_t index)
{
    if(index >= MOTOR_COUNT) return Motor_State_Stop;
    return stepper_motor[index].state;
}

Motor_Running_Type_t stepper_motor_get_running_type(uint8_t index)
{
    if(index >= MOTOR_COUNT) return Motor_Running_Type_Positioning;
    return stepper_motor[index].running_type;
}

void stepper_motor_set_position(uint8_t index, int position)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].position = position;
    stepper_motor[index].target_position = position;
    stepper_motor[index].state = Motor_State_Stop;
    
    stepper_motor_stop_timer(index);
}

int stepper_motor_get_position(uint8_t index)
{
    if(index >= MOTOR_COUNT) return 0;
    return stepper_motor[index].position;
}

void stepper_motor_set_target_position(uint8_t index, int target_position)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].target_position = target_position;
    stepper_motor[index].running_type = Motor_Running_Type_Positioning;
    
    if(target_position != stepper_motor[index].position) {
        stepper_motor_start_timer(index);
    }
}

void stepper_motor_stop(uint8_t index, bool motor_break, bool emergency)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].motor_break = motor_break;

    if(emergency) {
        stepper_motor[index].current_speed = 0;
        stepper_motor[index].accelerate_step = 0;
        stepper_motor[index].state = Motor_State_Stop;
        stepper_motor[index].target_direction = Motor_Direction_Stop;
        stepper_motor_stop_timer(index);
    } else {
        stepper_motor[index].state = Motor_State_Stopping;
        stepper_motor[index].accelerate_step = 0;
        stepper_motor[index].target_direction = Motor_Direction_Stop;
    }
}

void stepper_motor_set_direction(uint8_t index, Motor_Direction_t target_direction, uint16_t target_speed)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].running_type = Motor_Running_Type_Direction;
    stepper_motor[index].target_direction = target_direction;
    stepper_motor[index].target_speed = target_speed;
    
    if(target_direction != Motor_Direction_Stop) {
        stepper_motor_start_timer(index);
    }
}

void stepper_motor_set_speed_profile(uint8_t index, uint16_t target_speed, uint8_t accel_rate)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].target_speed = target_speed;
    stepper_motor[index].accelerate_rate = accel_rate;
}

void stepper_motor_set_sync_target(int nozzle_target, int base_target, uint16_t speed)
{
    stepper_motor_set_target_position(MOTOR_NECK, nozzle_target);
    stepper_motor_set_target_position(MOTOR_BASE, base_target);
    
    // 设置相同的速度配置
    for(int i = 0; i < MOTOR_COUNT; i++) {
        stepper_motor[i].target_speed = speed;
    }
}

bool stepper_motor_is_sync_complete(void)
{
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if(stepper_motor[i].position != stepper_motor[i].target_position) {
            return false;
        }
    }
    return true;
}

bool stepper_motor_all_stopped(void)
{
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if(stepper_motor[i].state != Motor_State_Stop) {
            return false;
        }
    }
    return true;
}

void stepper_motor_set_speed(uint8_t index, uint16_t target_speed, bool immediate)
{
    if(index < MOTOR_COUNT) {
        stepper_motor[index].target_speed = target_speed;
        if(immediate) {
            stepper_motor[index].current_speed = target_speed;
            stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
        }
    }
}

void stepper_motor_set_acceleration_rate(uint8_t index, uint8_t accel_rate)
{
    if(index < MOTOR_COUNT) {
        stepper_motor[index].accelerate_rate = accel_rate;
        stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
    }
}

uint16_t stepper_motor_get_current_speed(uint8_t index)
{
    if(index < MOTOR_COUNT) {
        return stepper_motor[index].current_speed;
    }
    return 0;
}

uint16_t stepper_motor_calc_accel_step(uint8_t index)
{
    if(stepper_motor[index].current_speed == 0) {
        return 0; // No acceleration
    } else if(stepper_motor[index].current_speed <= MOTOR_MIN_PPS) {
        return 1; // Minimum speed
    } else {
        return (stepper_motor[index].current_speed - MOTOR_MIN_PPS + stepper_motor[index].accelerate_rate - 1) / stepper_motor[index].accelerate_rate + 1;
    }
}

// --- 编码器数据获取函数实现 ---
bool stepper_motor_get_encoder_data(encoder_sampled_data_t *data, int timeout)
{
    if(encoder_sample_queue == NULL || data == NULL) {
        return false; // No queue initialized or invalid parameter
    }

    // Wait for data to be available in the queue
    BaseType_t result = xQueueReceive(encoder_sample_queue, data, timeout);
    return (result == pdTRUE);
}