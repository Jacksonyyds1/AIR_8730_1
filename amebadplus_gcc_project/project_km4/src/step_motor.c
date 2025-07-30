#include "step_motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "platform_autoconf.h"
#include "ameba_soc.h"

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
    
    // 硬件定时器相关
    uint8_t timer_idx;
    uint32_t timer_period;
} stepper_motor_t;

static QueueHandle_t encoder_sample_queue = NULL;

// --- 全局变量 ---
static stepper_motor_t stepper_motor[MOTOR_COUNT] = {
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        false, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        0, 0,
        MOTOR_TIMER_NECK_IDX, 0  // 硬件定时器索引
    }, // MOTOR_NECK
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        false, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        0, 0,
        MOTOR_TIMER_BASE_IDX, 0  // 硬件定时器索引
    }, // MOTOR_BASE
};

// GPIO对象
static gpio_t motor_gpio[MOTOR_COUNT][4];  // 每个电机4个GPIO引脚

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

// --- 硬件定时器中断处理函数 ---
u32 motor_neck_timer_handler(void *data)
{
    UNUSED(data);
    
    // 执行步进控制
    stepper_motor_driver(MOTOR_NECK);
    
    // 动态调整定时器周期
    stepper_motor_t *motor = &stepper_motor[MOTOR_NECK];
    if(motor->current_speed > 0) {
        uint32_t new_period = MOTOR_TIMER_FREQ / motor->current_speed - 1;
        if(new_period < MOTOR_MIN_PERIOD) new_period = MOTOR_MIN_PERIOD;
        if(new_period != motor->timer_period) {
            motor->timer_period = new_period;
            RTIM_ChangePeriod(TIMx[motor->timer_idx], new_period);
        }
    }
    
    // 如果电机停止且速度为0，停止定时器
    if(motor->state == Motor_State_Stop && motor->current_speed == 0) {
        RTIM_Cmd(TIMx[motor->timer_idx], DISABLE);
    }
    
    // 编码器采样
    if(motor->state != Motor_State_Stop) {
        encoder_sampled_data_t sample_data;
        sample_data.motor_index = MOTOR_NECK;
        sample_data.direction = motor->current_direction;
        sample_data.sampled_signal = encoder_neck_read();  // 需要实现此函数
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(encoder_sample_queue, &sample_data, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    
    RTIM_INTClear(TIMx[motor->timer_idx]);
    return 0;
}

u32 motor_base_timer_handler(void *data)
{
    UNUSED(data);
    
    // 执行步进控制
    stepper_motor_driver(MOTOR_BASE);
    
    // 动态调整定时器周期
    stepper_motor_t *motor = &stepper_motor[MOTOR_BASE];
    if(motor->current_speed > 0) {
        uint32_t new_period = MOTOR_TIMER_FREQ / motor->current_speed - 1;
        if(new_period < MOTOR_MIN_PERIOD) new_period = MOTOR_MIN_PERIOD;
        if(new_period != motor->timer_period) {
            motor->timer_period = new_period;
            RTIM_ChangePeriod(TIMx[motor->timer_idx], new_period);
        }
    }
    
    // 如果电机停止且速度为0，停止定时器
    if(motor->state == Motor_State_Stop && motor->current_speed == 0) {
        RTIM_Cmd(TIMx[motor->timer_idx], DISABLE);
    }
    
    // 编码器采样
    if(motor->state != Motor_State_Stop) {
        encoder_sampled_data_t sample_data;
        sample_data.motor_index = MOTOR_BASE;
        sample_data.direction = motor->current_direction;
        sample_data.sampled_signal = encoder_base_read();  // 需要实现此函数
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(encoder_sample_queue, &sample_data, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    
    RTIM_INTClear(TIMx[motor->timer_idx]);
    return 0;
}

// --- 静态函数声明 ---
static void stepper_motor_start_timer(uint8_t index);
static void stepper_motor_stop_timer(uint8_t index);
static void set_stepper_motor_output(uint8_t index, int position, bool motor_break);
static void stepper_motor_position_handler(stepper_motor_t *motor);
static void stepper_motor_direction_handler(stepper_motor_t *motor);
static void stepper_motor_driver(uint8_t index);
static uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate);

// --- 硬件定时器控制函数 ---
static void stepper_motor_start_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor_t *motor = &stepper_motor[index];
    
    // 计算初始定时器周期
    uint32_t period = (motor->current_speed > 0) ? 
                     (MOTOR_TIMER_FREQ / motor->current_speed - 1) : 
                     (MOTOR_TIMER_FREQ / MOTOR_MIN_PPS - 1);
                     
    if(period < MOTOR_MIN_PERIOD) period = MOTOR_MIN_PERIOD;
    motor->timer_period = period;
    
    // 更新定时器周期并启动
    RTIM_ChangePeriod(TIMx[motor->timer_idx], period);
    RTIM_Cmd(TIMx[motor->timer_idx], ENABLE);
}

static void stepper_motor_stop_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor_t *motor = &stepper_motor[index];
    RTIM_Cmd(TIMx[motor->timer_idx], DISABLE);
}

// --- GPIO输出控制函数 ---
static void set_stepper_motor_output(uint8_t index, int position, bool motor_break)
{
    if(index >= MOTOR_COUNT) return;
    
    uint8_t step = position & 0x07;  // 取模8
    uint8_t output = motor_break ? unipolar_step_sequence[step] : 0;
    
    // 设置4个GPIO引脚状态
    for(int i = 0; i < 4; i++) {
        gpio_write(&motor_gpio[index][i], (output >> i) & 0x01);
    }
}

// --- 位置控制处理函数 ---
static void stepper_motor_position_handler(stepper_motor_t *motor)
{
    if(motor->position == motor->target_position) {
        // 到达目标位置，开始减速停止
        motor->state = Motor_State_Stopping;
        motor->target_speed = 0;
        return;
    }
    
    // 确定方向和目标速度
    if(motor->target_position > motor->position) {
        motor->target_direction = Motor_Direction_Forward;
        motor->current_direction = Motor_Direction_Forward;
        motor->state = Motor_State_Forward;
        motor->position++;
    } else {
        motor->target_direction = Motor_Direction_Backward;
        motor->current_direction = Motor_Direction_Backward;
        motor->state = Motor_State_Backward;
        motor->position--;
    }
    
    // 计算距离并调整速度
    int distance = abs(motor->target_position - motor->position);
    uint16_t target_speed = (distance > ACCEL_DECEL_STEPS) ? 
                           MOTOR_MAX_PPS : 
                           (MOTOR_MIN_PPS + (distance * (MOTOR_MAX_PPS - MOTOR_MIN_PPS)) / ACCEL_DECEL_STEPS);
    
    motor->target_speed = target_speed;
    
    // 处理速度变化
    motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
}

// --- 方向控制处理函数 ---
static void stepper_motor_direction_handler(stepper_motor_t *motor)
{
    uint16_t target_speed = 0;
    
    switch(motor->state) {
    case Motor_State_Stop:
        if(motor->target_direction != Motor_Direction_Stop) {
            motor->state = Motor_State_Starting;
            motor->accelerate_step = 0;
        }
        break;
        
    case Motor_State_Starting:
        target_speed = motor->target_speed;
        if(motor->target_direction == Motor_Direction_Forward) {
            motor->position++;
            motor->state = Motor_State_Forward;
            motor->current_direction = Motor_Direction_Forward;
        } else if(motor->target_direction == Motor_Direction_Backward) {
            motor->position--;
            motor->state = Motor_State_Backward;
            motor->current_direction = Motor_Direction_Backward;
        }
        break;
        
    case Motor_State_Forward:
        if(motor->target_direction == Motor_Direction_Forward) {
            motor->position++;
            target_speed = motor->target_speed;
        } else {
            motor->state = Motor_State_Stopping;
            target_speed = 0;
        }
        break;
        
    case Motor_State_Backward:
        if(motor->target_direction == Motor_Direction_Backward) {
            motor->position--;
            target_speed = motor->target_speed;
        } else {
            motor->state = Motor_State_Stopping;
            target_speed = 0;
        }
        break;
        
    case Motor_State_Stopping:
        target_speed = 0;
        if(motor->current_speed <= MOTOR_MIN_PPS) {
            motor->current_direction = Motor_Direction_Stop;
            motor->state = Motor_State_Stop;
            motor->current_speed = 0;
            return;
        }
        break;
    }
    
    // 处理速度变化
    motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
}

// --- 速度控制函数 ---
static uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate)
{
    if(current_speed < target_speed) {
        // 加速
        if(current_speed == 0) {
            current_speed = MOTOR_MIN_PPS;
        } else if(target_speed - current_speed <= accelerate_rate) {
            current_speed = target_speed;
        } else {
            current_speed += accelerate_rate;
        }
    } else if(current_speed > target_speed) {
        // 减速
        if(current_speed - target_speed <= accelerate_rate) {
            current_speed = target_speed;
        } else {
            current_speed -= accelerate_rate;
        }
        
        if(current_speed < MOTOR_MIN_PPS && target_speed == 0) {
            current_speed = 0;
        }
    }
    
    return current_speed;
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

    // 设置电机输出
    set_stepper_motor_output(index, motor->position, 
                           (motor->state == Motor_State_Stop) ? motor->motor_break : true);
}

// --- 硬件定时器初始化 ---
void stepper_motor_timer_init(void)
{
    RTIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    // 启用定时器时钟
/*     RCC_PeriphClockCmd(APBPeriph_TIMx[MOTOR_TIMER_NECK_IDX], 
                       APBPeriph_TIMx_CLOCK[MOTOR_TIMER_NECK_IDX], ENABLE); */
    RCC_PeriphClockCmd(APBPeriph_TIMx[MOTOR_TIMER_BASE_IDX], 
                       APBPeriph_TIMx_CLOCK[MOTOR_TIMER_BASE_IDX], ENABLE);
    
/*     // 配置电机NECK硬件定时器
    RTIM_TimeBaseStructInit(&TIM_InitStruct);
    TIM_InitStruct.TIM_Idx = MOTOR_TIMER_NECK_IDX;
    TIM_InitStruct.TIM_Period = MOTOR_TIMER_FREQ / MOTOR_MIN_PPS - 1;  // 初始周期
    TIM_InitStruct.TIM_Prescaler = 0;  // 无预分频，使用40MHz
    
    RTIM_TimeBaseInit(TIMx[MOTOR_TIMER_NECK_IDX], &TIM_InitStruct, 
                      TIMx_irq[MOTOR_TIMER_NECK_IDX], 
                      (IRQ_FUN)motor_neck_timer_handler, 0);
    RTIM_INTConfig(TIMx[MOTOR_TIMER_NECK_IDX], TIM_IT_Update, ENABLE);
     */
    // 配置电机BASE硬件定时器
    RTIM_TimeBaseStructInit(&TIM_InitStruct);
    TIM_InitStruct.TIM_Idx = MOTOR_TIMER_BASE_IDX;
    TIM_InitStruct.TIM_Period = MOTOR_TIMER_FREQ / MOTOR_MIN_PPS - 1;  // 初始周期
    TIM_InitStruct.TIM_Prescaler = 0;  // 无预分频，使用40MHz
    
    RTIM_TimeBaseInit(TIMx[MOTOR_TIMER_BASE_IDX], &TIM_InitStruct, 
                      TIMx_irq[MOTOR_TIMER_BASE_IDX], 
                      (IRQ_FUN)motor_base_timer_handler, 0);
    RTIM_INTConfig(TIMx[MOTOR_TIMER_BASE_IDX], TIM_IT_Update, ENABLE);
    
    printf("Hardware timers initialized for stepper motors\n");
}

// --- GPIO初始化 ---
void stepper_motor_gpio_init(void)
{
    // MOTOR_NECK GPIO初始化
   /*  gpio_init(&motor_gpio[MOTOR_NECK][0], MOTOR_NECK_A_PLUS);
    gpio_init(&motor_gpio[MOTOR_NECK][1], MOTOR_NECK_B_PLUS);
    gpio_init(&motor_gpio[MOTOR_NECK][2], MOTOR_NECK_A_MINUS);
    gpio_init(&motor_gpio[MOTOR_NECK][3], MOTOR_NECK_B_MINUS);
    
    for(int i = 0; i < 4; i++) {
        gpio_dir(&motor_gpio[MOTOR_NECK][i], PIN_OUTPUT);
        gpio_mode(&motor_gpio[MOTOR_NECK][i], PullNone);
        gpio_write(&motor_gpio[MOTOR_NECK][i], 0);
    } */
    
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
    
    printf("Stepper motor GPIO initialized\n");
}

// --- 公共API函数实现 ---

void stepper_motor_init(void)
{
    // 创建编码器采样队列
    encoder_sample_queue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(encoder_sampled_data_t));
    if(encoder_sample_queue == NULL) {
        printf("Failed to create encoder sample queue\n");
        return;
    }
    
    // 初始化GPIO
    stepper_motor_gpio_init();
    
    // 初始化硬件定时器
    stepper_motor_timer_init();
    
    printf("Stepper motor system initialized with hardware timers\n");
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
    }
    stepper_motor[index].target_direction = Motor_Direction_Stop;
}

void stepper_motor_set_direction(uint8_t index, Motor_Direction_t target_direction, uint16_t target_speed)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].target_direction = target_direction;
    stepper_motor[index].target_speed = target_speed;
    stepper_motor[index].running_type = Motor_Running_Type_Direction;
    
    if(stepper_motor[index].state == Motor_State_Stop && target_direction != Motor_Direction_Stop) {
        stepper_motor_start_timer(index);
    }
}

void stepper_motor_set_speed(uint8_t index, uint16_t target_speed, bool immediate)
{
    if(index >= MOTOR_COUNT) return;
    
    if(target_speed < MOTOR_MIN_PPS) {
        stepper_motor_stop(index, false, immediate);
        return;
    }
    
    stepper_motor[index].target_speed = target_speed;
    if(immediate) {
        stepper_motor[index].current_speed = target_speed;
        stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
    }
}

void stepper_motor_set_acceleration_rate(uint8_t index, uint8_t accel_rate)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].accelerate_rate = accel_rate;
    stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
}

uint16_t stepper_motor_get_current_speed(uint8_t index)
{
    if(index >= MOTOR_COUNT) return 0;
    return stepper_motor[index].current_speed;
}

uint16_t stepper_motor_calc_accel_step(uint8_t index)
{
    if(index >= MOTOR_COUNT) return 0;
    
    if(stepper_motor[index].current_speed == 0) {
        return 0;
    } else if(stepper_motor[index].current_speed <= MOTOR_MIN_PPS) {
        return 1;
    } else {
        return (stepper_motor[index].current_speed - MOTOR_MIN_PPS + 
                stepper_motor[index].accelerate_rate - 1) / stepper_motor[index].accelerate_rate + 1;
    }
}

bool stepper_motor_get_encoder_data(encoder_sampled_data_t *data, int timeout)
{
    if(encoder_sample_queue == NULL || data == NULL) {
        return false;
    }

    BaseType_t result = xQueueReceive(encoder_sample_queue, data, timeout);
    return (result == pdTRUE);
}

// --- 同步控制函数 ---
void stepper_motor_set_sync_target(int neck_target, int base_target, uint16_t speed)
{
    stepper_motor_set_target_position(MOTOR_NECK, neck_target);
    stepper_motor_set_target_position(MOTOR_BASE, base_target);
    
    stepper_motor_set_speed(MOTOR_NECK, speed, false);
    stepper_motor_set_speed(MOTOR_BASE, speed, false);
}

bool stepper_motor_is_sync_complete(void)
{
    return (stepper_motor_get_state(MOTOR_NECK) == Motor_State_Stop &&
            stepper_motor_get_state(MOTOR_BASE) == Motor_State_Stop);
}

bool stepper_motor_all_stopped(void)
{
    for(uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if(stepper_motor_get_state(i) != Motor_State_Stop) {
            return false;
        }
    }
    return true;
}

void stepper_motor_set_speed_profile(uint8_t index, uint16_t target_speed, uint8_t accel_rate)
{
    stepper_motor_set_speed(index, target_speed, false);
    stepper_motor_set_acceleration_rate(index, accel_rate);
}