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

// GPIO对象
static gpio_t motor_gpio[MOTOR_COUNT][4];  // 每个电机4个GPIO引脚
static gtimer_t motor_timer[MOTOR_COUNT];   // 每个电机一个定时器

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

// 定时器回调函数
static void timer_neck_handler(void);
static void timer_base_handler(void);

// --- GPIO初始化 ---
void stepper_motor_gpio_init(void)
{

        // 先将引脚配置为GPIO功能，取消SPI功能
    printf("GPIO init: Configuring pins as GPIO...\n");

    // MOTOR_NECK 引脚
    Pinmux_Config(MOTOR_NECK_A_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_B_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_A_MINUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_NECK_B_MINUS, PINMUX_FUNCTION_GPIO);

    // MOTOR_BASE 引脚
    Pinmux_Config(MOTOR_BASE_A_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_B_PLUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_A_MINUS, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_BASE_B_MINUS, PINMUX_FUNCTION_GPIO);

     printf("GPIO init: Pins configured as GPIO\n");

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
     printf("GPIO init: Completed successfully\n");
}

// --- 定时器初始化 ---
void stepper_motor_timer_init(void)
{
    printf("Timer init: Enabling timer clocks...\n");

    // 初始化MOTOR_NECK定时器
/*     gtimer_init(&motor_timer[MOTOR_NECK], MOTOR_TIMER_NOZZLE);
    printf("Timer init: MOTOR_NECK timer initialized\n"); */
    
    // 初始化MOTOR_BASE定时器  
    gtimer_init(&motor_timer[MOTOR_BASE], MOTOR_TIMER_BASE);
    printf("Timer init: MOTOR_BASE timer initialized\n");
    
    printf("Timer init completed successfully\n");
}

// --- 定时器启动函数 ---
static void stepper_motor_start_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    // 计算定时器周期 (单位: 微秒)
    uint32_t period_us = (STEP_TICKS_MAX - stepper_motor[index].current_speed)*10 ; // 10us基准
    if(period_us < 100) period_us = 100; // 最小100us，避免过快
    
    if(index == MOTOR_NECK) {
        gtimer_start_periodical(&motor_timer[MOTOR_NECK], period_us,
                               (void*)timer_neck_handler, MOTOR_NECK);
    } else if(index == MOTOR_BASE) {
        gtimer_start_periodical(&motor_timer[MOTOR_BASE], period_us,
                               (void*)timer_base_handler, MOTOR_BASE);
    }
}

// --- 定时器停止函数 ---
static void stepper_motor_stop_timer(uint8_t index)
{
    if(index >= MOTOR_COUNT) return;
    
    gtimer_stop(&motor_timer[index]);
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
        // 关闭所有输出
        for(int i = 0; i < 4; i++) {
            gpio_write(&motor_gpio[index][i], 0);
        }
    }
}

// --- 速度处理函数 ---
static uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate)
{
    if (current_speed < target_speed) {
        // 加速
        if (target_speed - current_speed <= accelerate_rate) {
            current_speed = target_speed;
        } else {
            current_speed += accelerate_rate;
        }
    } else if (current_speed > target_speed) {
        // 减速
        if (current_speed - target_speed <= accelerate_rate) {
            current_speed = target_speed;
        } else {
            current_speed -= accelerate_rate;
        }
    }
    
    return current_speed;
}

// --- 位置控制处理 ---
static void stepper_motor_position_handler(stepper_motor_t *motor)
{
    uint16_t target_speed = 0;
    
    switch(motor->state) {
        case Motor_State_Stop:
            if(motor->position != motor->target_position) {
                motor->state = Motor_State_Starting;
                motor->accelerate_step = 0;
                motor->current_speed = 0;
            }
            break;

        case Motor_State_Starting:
            if(motor->position < motor->target_position) {
                motor->position++;
                motor->state = Motor_State_Forward;
                motor->current_direction = Motor_Direction_Forward;
            } else if(motor->position > motor->target_position) {
                motor->position--;
                motor->state = Motor_State_Backward;
                motor->current_direction = Motor_Direction_Backward;
            }
            target_speed = motor->target_speed;
            break;

        case Motor_State_Forward:
            motor->position++;
            if (motor->position < motor->target_position) {
                if ((motor->target_position - motor->position) <= motor->accelerate_step) {
                    motor->state = Motor_State_Stopping;
                } else {
                    target_speed = motor->target_speed;
                }
            } else {
                motor->state = Motor_State_Stopping;
            }
            break;

        case Motor_State_Backward:
            motor->position--;
            if (motor->position > motor->target_position) {
                if ((motor->position - motor->target_position) <= motor->accelerate_step) {
                    motor->state = Motor_State_Stopping;
                } else {
                    target_speed = motor->target_speed;
                }
            } else {
                motor->state = Motor_State_Stopping;
            }
            break;

        case Motor_State_Stopping:
            if(motor->accelerate_step > 0) {
                if(motor->current_direction == Motor_Direction_Forward) {
                    motor->position++;
                } else {
                    motor->position--;
                }
            } else {
                motor->state = Motor_State_Stop;
                motor->current_speed = 0;
                motor->accelerate_step = 0;
            }
            break;

        default:
            break;
    }

    // 速度控制
    if (target_speed > motor->current_speed) {
        motor->accelerate_step++;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    } else if (target_speed < motor->current_speed) {
        if(motor->accelerate_step > 0) motor->accelerate_step--;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    }
}

// --- 方向控制处理 ---
static void stepper_motor_direction_handler(stepper_motor_t *motor)
{
    uint16_t target_speed = 0;
    
    switch(motor->target_direction) {
        case Motor_Direction_Stop:
            if(motor->current_direction == Motor_Direction_Forward) {
                motor->position++;
            } else if(motor->current_direction == Motor_Direction_Backward) {
                motor->position--;
            }

            if(motor->accelerate_step > 0) {
                motor->state = Motor_State_Stopping;
            } else {
                motor->state = Motor_State_Stop;
                motor->current_direction = Motor_Direction_Stop;
                motor->current_speed = 0;
                motor->accelerate_step = 0;
            }
            break;

        case Motor_Direction_Forward:
            if(motor->current_direction == Motor_Direction_Backward) {
                if(motor->accelerate_step == 0) {
                    motor->current_direction = Motor_Direction_Stop;
                    motor->state = Motor_State_Stop;
                } else {
                    motor->state = Motor_State_Stopping;
                    motor->position--;
                }
            } else {
                motor->position++;
                motor->state = Motor_State_Forward;
                motor->current_direction = Motor_Direction_Forward;
                target_speed = motor->target_speed;
            }
            break;
        
        case Motor_Direction_Backward:
            if(motor->current_direction == Motor_Direction_Forward) {
                if(motor->accelerate_step == 0) {
                    motor->current_direction = Motor_Direction_Stop;
                    motor->state = Motor_State_Stop;
                } else {
                    motor->state = Motor_State_Stopping;
                    motor->position++;
                }
            } else {
                motor->position--;
                motor->state = Motor_State_Backward;
                motor->current_direction = Motor_Direction_Backward;
                target_speed = motor->target_speed;
            }
            break;
    }

    // 速度控制
    if (target_speed > motor->current_speed) {
        motor->accelerate_step++;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    } else if (target_speed < motor->current_speed) {
        if(motor->accelerate_step > 0) motor->accelerate_step--;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
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
static void timer_neck_handler(void)
{
    // 执行步进控制
    stepper_motor_driver(MOTOR_NECK);
    
    // 动态调整定时器周期
    uint32_t new_period_us = (STEP_TICKS_MAX - stepper_motor[MOTOR_NECK].current_speed) * 10;
    if(new_period_us < 100) new_period_us = 100;
    
    // 如果电机停止且速度为0，停止定时器
    if(stepper_motor[MOTOR_NECK].state == Motor_State_Stop &&
       stepper_motor[MOTOR_NECK].current_speed == 0) {
        stepper_motor_stop_timer(MOTOR_NECK);
    } else {
        // 重新启动定时器以调整周期
        gtimer_stop(&motor_timer[MOTOR_NECK]);
        gtimer_start_periodical(&motor_timer[MOTOR_NECK], new_period_us,
                               (void*)timer_neck_handler, MOTOR_NECK);
    }
}

static void timer_base_handler(void)
{
    // 执行步进控制
    stepper_motor_driver(MOTOR_BASE);
    
    // 动态调整定时器周期
    uint32_t new_period_us = (STEP_TICKS_MAX - stepper_motor[MOTOR_BASE].current_speed) * 10;
    if(new_period_us < 100) new_period_us = 100;
    
    // 如果电机停止且速度为0，停止定时器
    if(stepper_motor[MOTOR_BASE].state == Motor_State_Stop && 
       stepper_motor[MOTOR_BASE].current_speed == 0) {
        stepper_motor_stop_timer(MOTOR_BASE);
    } else {
        // 重新启动定时器以调整周期
        gtimer_stop(&motor_timer[MOTOR_BASE]);
        gtimer_start_periodical(&motor_timer[MOTOR_BASE], new_period_us,
                               (void*)timer_base_handler, MOTOR_BASE);
    }
}

// --- 公共API函数实现 ---

void stepper_motor_init(void)
{
    // 初始化GPIO
    stepper_motor_gpio_init();
    
    // 初始化定时器
    stepper_motor_timer_init();
    
    printf("Stepper motor system initialized on RTL8721DCM\n");
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

void stepper_motor_stop(uint8_t index, bool motor_break,bool emergency)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].motor_break = motor_break;

    if(emergency){
        stepper_motor[index].current_speed = 0;
        stepper_motor[index].accelerate_step = 0;
        stepper_motor[index].state = Motor_State_Stop;
        stepper_motor[index].target_direction = Motor_Direction_Stop;

    }
    else{
        stepper_motor[index].state = Motor_State_Stopping;
        stepper_motor[index].accelerate_step = 0;
    }
    stepper_motor[index].target_direction = Motor_Direction_Stop;

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

void stepper_motor_set_sync_target(int neck_target, int base_target, uint16_t speed)
{
    stepper_motor[MOTOR_NECK].running_type = Motor_Running_Type_Positioning;
    stepper_motor[MOTOR_NECK].target_position = neck_target;
    stepper_motor[MOTOR_NECK].target_speed = speed;

    stepper_motor[MOTOR_BASE].running_type = Motor_Running_Type_Positioning;
    stepper_motor[MOTOR_BASE].target_position = base_target;
    stepper_motor[MOTOR_BASE].target_speed = speed;

    if(neck_target != stepper_motor[MOTOR_NECK].position) {
        stepper_motor_start_timer(MOTOR_NECK);
    }
    if(base_target != stepper_motor[MOTOR_BASE].position) {
        stepper_motor_start_timer(MOTOR_BASE);
    }
}

bool stepper_motor_is_sync_complete(void)
{
    return (stepper_motor[MOTOR_NECK].state == Motor_State_Stop &&
            stepper_motor[MOTOR_BASE].state == Motor_State_Stop &&
            stepper_motor[MOTOR_NECK].position == stepper_motor[MOTOR_NECK].target_position &&
            stepper_motor[MOTOR_BASE].position == stepper_motor[MOTOR_BASE].target_position);
}

bool stepper_motor_all_stopped(void)
{
    return (stepper_motor[MOTOR_NECK].state == Motor_State_Stop &&
            stepper_motor[MOTOR_BASE].state == Motor_State_Stop);
}

void stepper_motor_set_speed_profile(uint8_t index, uint16_t max_speed, uint8_t accel_rate)
{
    if(index >= MOTOR_COUNT) return;
    
    stepper_motor[index].target_speed = max_speed;
    stepper_motor[index].accelerate_rate = accel_rate;
}

void stepper_motor_set_speed(uint8_t index, uint16_t target_speed, bool immediate)
{
    if(target_speed < MOTOR_MIN_PPS)
    {
        stepper_motor_stop(index, false, immediate);
        return; // Invalid speed, stop motor
    }

    if(index < MOTOR_COUNT)
    {
        stepper_motor[index]. target_speed = target_speed;
        if(immediate)
        {
            stepper_motor[index].target_speed = target_speed;
            stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
        }
    }
}
void stepper_motor_set_acceleration_rate(uint8_t index, uint8_t accel_rate)
{
    if(index < MOTOR_COUNT)
    {
        stepper_motor[index].accelerate_rate = accel_rate;
        stepper_motor[index].accelerate_step = stepper_motor_calc_accel_step(index);
    }
}
uint16_t stepper_motor_get_current_speed(uint8_t index)
{
    if(index < MOTOR_COUNT)
    {
        return stepper_motor[index].current_speed;
    }
    return 0;
}
uint16_t stepper_motor_calc_accel_step(uint8_t index)
{
    if(stepper_motor[index].current_speed == 0)
    {
        return 0; // No acceleration
    }
    else if(stepper_motor[index].current_speed <= MOTOR_MIN_PPS)
    {
        return 1; // Minimum speed
    }
    else
{
    return (stepper_motor[index].current_speed - MOTOR_MIN_PPS + stepper_motor[index].accelerate_rate - 1) / stepper_motor[index].accelerate_rate + 1;
    }
}
// --- 编码器数据获取函数实现 ---
bool stepper_motor_get_encoder_data(encoder_sampled_data_t *data, int timeout)
{
    if(encoder_sample_queue == NULL || data == NULL)
    {
        return false; // No queue initialized or invalid parameter
    }

    // Wait for data to be available in the queue
    BaseType_t result = xQueueReceive(encoder_sample_queue, data, timeout);
    return (result == pdTRUE);
}
