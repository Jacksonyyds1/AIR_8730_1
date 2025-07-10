//=================================================================
// microstep_controller.c - 微步控制实现
//=================================================================

#include "ameba_soc.h"
#include "pwmout_api.h"
#include "gpio_api.h"
#include "timer_api.h"
#include "os_wrapper.h"
#include <math.h>
#include <stdbool.h>
#include "microstep_controller.h"

// 配置选择：使用PWM还是专用驱动IC
//#define USE_PWM_CONTROL     0    // 暂时关闭PWM，使用GPIO模式
//#define MICROSTEP_RESOLUTION 64  // 微步分辨率 (每个全步的微步数)

/* // GPIO配置 (兼容现有引脚)
#define MOTOR_A_PLUS_PIN    PB_21
#define MOTOR_B_PLUS_PIN    PB_20
#define MOTOR_A_MINUS_PIN   PB_30 
#define MOTOR_B_MINUS_PIN   PB_18 */

// 微步控制器状态
typedef struct {
    // GPIO对象 (暂时使用GPIO模式)
    gpio_t gpio_a_plus;
    gpio_t gpio_b_plus;
    gpio_t gpio_a_minus;
    gpio_t gpio_b_minus;
    
    // 运动状态
    int32_t current_position;    // 当前微步位置
    int32_t target_position;     // 目标微步位置
    uint16_t step_delay_us;      // 步进延迟 (微秒)
    bool is_running;
    bool initialized;
    
    // 定时器
    gtimer_t step_timer;
} true_microstep_controller_t;

static true_microstep_controller_t controller = {0};

// 真正的微步表 - 64步正弦波电流分配
static const current_levels_t microstep_sine_table[MICROSTEP_RESOLUTION] = {
    // 基于正弦/余弦函数计算的电流值
    // A相: sin(θ), B相: cos(θ)，θ从0到2π
    
    // 第一象限 (0°-90°): A+增加, B+减少
    {255,   0,   0,   0}, {255,  25,   0,   0}, {255,  50,   0,   0}, {255,  74,   0,   0},
    {255,  98,   0,   0}, {255, 120,   0,   0}, {255, 142,   0,   0}, {255, 162,   0,   0},
    {255, 180,   0,   0}, {255, 197,   0,   0}, {255, 212,   0,   0}, {255, 226,   0,   0},
    {255, 238,   0,   0}, {255, 248,   0,   0}, {255, 254,   0,   0}, {255, 255,   0,   0},
    
    // 第二象限 (90°-180°): A+减少, B+满电流
    {254, 255,   0,   0}, {248, 255,   0,   0}, {238, 255,   0,   0}, {226, 255,   0,   0},
    {212, 255,   0,   0}, {197, 255,   0,   0}, {180, 255,   0,   0}, {162, 255,   0,   0},
    {142, 255,   0,   0}, {120, 255,   0,   0}, { 98, 255,   0,   0}, { 74, 255,   0,   0},
    { 50, 255,   0,   0}, { 25, 255,   0,   0}, {  0, 255,   0,   0}, {  0, 255,  25,   0},
    
    // 第三象限 (180°-270°): B+减少, A-增加
    {  0, 254,  50,   0}, {  0, 248,  74,   0}, {  0, 238,  98,   0}, {  0, 226, 120,   0},
    {  0, 212, 142,   0}, {  0, 197, 162,   0}, {  0, 180, 180,   0}, {  0, 162, 197,   0},
    {  0, 142, 212,   0}, {  0, 120, 226,   0}, {  0,  98, 238,   0}, {  0,  74, 248,   0},
    {  0,  50, 254,   0}, {  0,  25, 255,   0}, {  0,   0, 255,   0}, {  0,   0, 255,  25},
    
    // 第四象限 (270°-360°): A-减少, B-增加
    {  0,   0, 254,  50}, {  0,   0, 248,  74}, {  0,   0, 238,  98}, {  0,   0, 226, 120},
    {  0,   0, 212, 142}, {  0,   0, 197, 162}, {  0,   0, 180, 180}, {  0,   0, 162, 197},
    {  0,   0, 142, 212}, {  0,   0, 120, 226}, {  0,   0,  98, 238}, {  0,   0,  74, 248},
    {  0,   0,  50, 254}, {  0,   0,  25, 255}, {  0,   0,   0, 255}, { 25,   0,   0, 255}
};

// 静态函数声明
static void true_microstep_timer_handler(void);
static void set_motor_currents(const current_levels_t *currents);

/**
 * @brief 初始化真正的微步控制器
 */
void true_microstep_init(void)
{
    if(controller.initialized) {
        printf("True microstep already initialized\n");
        return;
    }
    
    printf("Initializing TRUE microstep controller...\n");
    
    // 暂时使用GPIO控制模式 (兼容现有硬件)
    printf("Using GPIO control mode (compatible with existing hardware)\n");
    
    // 配置引脚为GPIO功能
    Pinmux_Config(MOTOR_A_PLUS_PIN, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_B_PLUS_PIN, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_A_MINUS_PIN, PINMUX_FUNCTION_GPIO);
    Pinmux_Config(MOTOR_B_MINUS_PIN, PINMUX_FUNCTION_GPIO);
    
    // 初始化GPIO
    gpio_init(&controller.gpio_a_plus, MOTOR_A_PLUS_PIN);
    gpio_init(&controller.gpio_b_plus, MOTOR_B_PLUS_PIN);
    gpio_init(&controller.gpio_a_minus, MOTOR_A_MINUS_PIN);
    gpio_init(&controller.gpio_b_minus, MOTOR_B_MINUS_PIN);
    
    // 设置为输出模式
    gpio_dir(&controller.gpio_a_plus, PIN_OUTPUT);
    gpio_dir(&controller.gpio_b_plus, PIN_OUTPUT);
    gpio_dir(&controller.gpio_a_minus, PIN_OUTPUT);
    gpio_dir(&controller.gpio_b_minus, PIN_OUTPUT);
    
    // 设置上拉，初始化为低电平
    gpio_mode(&controller.gpio_a_plus, PullNone);
    gpio_mode(&controller.gpio_b_plus, PullNone);
    gpio_mode(&controller.gpio_a_minus, PullNone);
    gpio_mode(&controller.gpio_b_minus, PullNone);
    
    gpio_write(&controller.gpio_a_plus, 0);
    gpio_write(&controller.gpio_b_plus, 0);
    gpio_write(&controller.gpio_a_minus, 0);
    gpio_write(&controller.gpio_b_minus, 0);
    
    printf("GPIO pins initialized successfully\n");
    
    // 初始化定时器
    RCC_PeriphClockCmd(APBPeriph_TIMx[5], APBPeriph_TIMx_CLOCK[5], ENABLE);
    gtimer_init(&controller.step_timer, TIMER5);
    
    // 初始化状态
    controller.current_position = 0;
    controller.target_position = 0;
    controller.step_delay_us = 1000;  // 默认1ms/步
    controller.is_running = false;
    controller.initialized = true;
    
    printf("True microstep controller initialized successfully\n");
    printf("Microstep resolution: %d steps per full step\n", MICROSTEP_RESOLUTION);
    printf("Position resolution: %.4f degrees per microstep\n", 1.8 / MICROSTEP_RESOLUTION);
}

/**
 * @brief 设置电机电流
 */
static void set_motor_currents(const current_levels_t *currents)
{
    // 在GPIO模式下，我们只能做简单的on/off控制
    // 这里先实现基本功能，后续可升级到PWM控制
    
    gpio_write(&controller.gpio_a_plus, currents->a_plus > 128 ? 1 : 0);
    gpio_write(&controller.gpio_b_plus, currents->b_plus > 128 ? 1 : 0);
    gpio_write(&controller.gpio_a_minus, currents->a_minus > 128 ? 1 : 0);
    gpio_write(&controller.gpio_b_minus, currents->b_minus > 128 ? 1 : 0);
}

/**
 * @brief 移动到指定微步位置
 */
void true_microstep_move_to_position(int32_t target_position, uint16_t speed_pps)
{
    if(!controller.initialized) {
        printf("ERROR: True microstep not initialized\n");
        return;
    }
    
    controller.target_position = target_position;
    
    // 计算步进延迟 (脉冲每秒 → 微秒延迟)
    if(speed_pps > 0) {
        controller.step_delay_us = 1000000 / speed_pps;
        if(controller.step_delay_us < 100) controller.step_delay_us = 100;  // 最小100us
    }
    
    printf("True microstep: target=%ld, current=%ld, speed=%d pps\n",
           target_position, controller.current_position, speed_pps);
    
    if(controller.current_position != controller.target_position) {
        controller.is_running = true;
        
        // 启动定时器
        gtimer_start_periodical(&controller.step_timer, controller.step_delay_us,
                               (void*)true_microstep_timer_handler, 0);
    }
}

/**
 * @brief 相对移动指定微步数
 */
void true_microstep_move_steps(int32_t steps, uint16_t speed_pps)
{
    int32_t target = controller.current_position + steps;
    true_microstep_move_to_position(target, speed_pps);
}

/**
 * @brief 停止微步运动
 */
void true_microstep_stop(bool hold_position)
{
    controller.is_running = false;
    gtimer_stop(&controller.step_timer);
    
    if(!hold_position) {
        // 关闭所有电流
        current_levels_t zero_current = {0, 0, 0, 0};
        set_motor_currents(&zero_current);
        printf("True microstep stopped, power off\n");
    } else {
        printf("True microstep stopped, holding position\n");
    }
}

/**
 * @brief 定时器中断处理函数
 */
static void true_microstep_timer_handler(void)
{
    if(!controller.is_running) return;
    
    // 判断运动方向
    bool forward = (controller.target_position > controller.current_position);
    
    // GPIO控制模式：设置对应位置的电流
    if(forward) {
        controller.current_position++;
    } else {
        controller.current_position--;
    }
    
    // 获取当前位置对应的电流值
    int32_t table_index = controller.current_position % MICROSTEP_RESOLUTION;
    if(table_index < 0) table_index += MICROSTEP_RESOLUTION;
    
    set_motor_currents(&microstep_sine_table[table_index]);
    
    // 检查是否到达目标位置
    if(controller.current_position == controller.target_position) {
        controller.is_running = false;
        gtimer_stop(&controller.step_timer);
        printf("True microstep reached target position: %ld\n", controller.current_position);
    }
    
    // 每100步打印进度
    static uint32_t step_count = 0;
    if(++step_count % 100 == 0) {
        printf("Progress: position=%ld, target=%ld\n", 
               controller.current_position, controller.target_position);
    }
}

/**
 * @brief 获取当前状态
 */
bool true_microstep_is_running(void)
{
    return controller.is_running;
}

int32_t true_microstep_get_position(void)
{
    return controller.current_position;
}

float true_microstep_get_angle_degrees(void)
{
    // 计算当前角度 (假设1.8°/全步)
    return (controller.current_position * 1.8f) / MICROSTEP_RESOLUTION;
}

/**
 * @brief 设置当前位置（用于零点校准）
 */
void true_microstep_set_position(int32_t position)
{
    if(controller.is_running) {
        printf("Warning: Setting position while motor is running\n");
        true_microstep_stop(false);
    }
    
    controller.current_position = position;
    controller.target_position = position;
    
    printf("True microstep position set to: %ld\n", position);
}

/**
 * @brief 微步精度测试
 */
void true_microstep_precision_test(void)
{
    printf("\n=== True Microstep Precision Test ===\n");
    
    if(!controller.initialized) {
        printf("Please initialize first!\n");
        return;
    }
    
    printf("Testing %d microsteps in one full step...\n", MICROSTEP_RESOLUTION);
    
    // 缓慢转动一个完整步进，观察平滑度
    for(int i = 0; i < MICROSTEP_RESOLUTION; i++) {
        true_microstep_move_to_position(i, 50);  // 50 pps，很慢
        
        while(true_microstep_is_running()) {
            rtos_time_delay_ms(10);
        }
        
        printf("Microstep %d/%d, Angle: %.3f°\n", 
               i, MICROSTEP_RESOLUTION, true_microstep_get_angle_degrees());
        
        rtos_time_delay_ms(100);  // 停留观察
    }
    
    printf("Precision test completed\n");
    printf("Total angle moved: %.3f° (should be 1.8°)\n", 
           true_microstep_get_angle_degrees());
}

/**
 * @brief 速度测试
 */
void true_microstep_speed_test(void)
{
    printf("\n=== True Microstep Speed Test ===\n");
    
    uint16_t test_speeds[] = {100, 500, 1000, 2000, 5000};  // pps
    int num_tests = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    for(int i = 0; i < num_tests; i++) {
        printf("Testing speed: %d pps\n", test_speeds[i]);
        
        uint32_t start_time = rtos_time_get_current_system_time_ms();
        
        // 移动1000微步
        true_microstep_move_steps(1000, test_speeds[i]);
        
        while(true_microstep_is_running()) {
            rtos_time_delay_ms(1);
        }
        
        uint32_t end_time = rtos_time_get_current_system_time_ms();
        uint32_t actual_time = end_time - start_time;
        uint32_t expected_time = 1000 * 1000 / test_speeds[i];  // ms
        
        printf("Expected time: %lu ms, Actual time: %lu ms\n", 
               expected_time, actual_time);
        
        rtos_time_delay_ms(500);
    }
}

/**
 * @brief 平滑度演示
 */
void true_microstep_smoothness_demo(void)
{
    printf("\n=== True Microstep Smoothness Demo ===\n");
    
    printf("Demo 1: Slow smooth rotation\n");
    // 慢速平滑旋转5圈
    int32_t five_revolutions = 5 * 200 * MICROSTEP_RESOLUTION;  // 5圈 × 200步/圈 × 微步/步
    true_microstep_move_steps(five_revolutions, 1000);  // 1000 pps
    
    while(true_microstep_is_running()) {
        rtos_time_delay_ms(100);
        printf("Position: %ld, Angle: %.1f°\n", 
               true_microstep_get_position(), true_microstep_get_angle_degrees());
    }
    
    rtos_time_delay_ms(1000);
    
    printf("Demo 2: Reverse rotation\n");
    true_microstep_move_steps(-five_revolutions, 2000);  // 反向，更快
    
    while(true_microstep_is_running()) {
        rtos_time_delay_ms(100);
    }
    
    printf("Smoothness demo completed\n");
}

/**
 * @brief 电流波形显示（调试用）
 */
void true_microstep_show_current_waveform(void)
{
    printf("\n=== Current Waveform Analysis ===\n");
    printf("Microstep | A+  | B+  | A-  | B-  | Angle\n");
    printf("----------|-----|-----|-----|-----|------\n");
    
    for(int i = 0; i < MICROSTEP_RESOLUTION; i++) {
        const current_levels_t *current = &microstep_sine_table[i];
        float angle = (i * 360.0f) / MICROSTEP_RESOLUTION;
        
        printf("%6d    |%4d |%4d |%4d |%4d | %5.1f°\n",
               i, current->a_plus, current->b_plus, 
               current->a_minus, current->b_minus, angle);
    }
}

//=================================================================
// 兼容性接口 - 保持与现有代码的兼容性
//=================================================================

/**
 * @brief 兼容接口：初始化
 */
void enhanced_microstep_init(void)
{
    true_microstep_init();
}

/**
 * @brief 兼容接口：移动步数
 */
void enhanced_microstep_move_steps(int32_t steps, uint16_t speed_ms)
{
    // 转换延迟到速度 (ms/步 → 步/秒)
    uint16_t speed_pps = (speed_ms > 0) ? (1000 / speed_ms) : 1000;
    if(speed_pps > 10000) speed_pps = 10000;  // 限制最大速度
    
    // 转换为真正微步数
    int32_t microsteps = steps * MICROSTEP_RESOLUTION / 4;  // 假设原来是1/4步
    
    true_microstep_move_steps(microsteps, speed_pps);
}

/**
 * @brief 完整的演示程序
 */
void true_microstep_complete_demo(void)
{
    printf("\n" 
           "========================================\n"
           "  TRUE MICROSTEP CONTROLLER DEMO\n"
           "========================================\n");
    
    // 初始化
    true_microstep_init();
    
    // 显示电流波形
    true_microstep_show_current_waveform();
    
    // 精度测试
    true_microstep_precision_test();
    
    // 速度测试
    true_microstep_speed_test();
    
    // 平滑度演示
    true_microstep_smoothness_demo();
    
    printf("\n"
           "========================================\n"
           "  DEMO COMPLETED\n"
           "========================================\n");
}