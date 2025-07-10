#include "task_manager.h"
#include "multi_button.h"
#include "Lcd.h"
#include "step_motor.h"
#include "gpio_api.h"  
#include <stdio.h>

/*================================ GPIO配置定义 ==============================*/
// 按键GPIO引脚定义 - 根据实际硬件连接修改
#define POWER_BUTTON_PIN        _PA_12    // 电源按键引脚
#define CONTROL_BUTTON_PIN      _PA_28    // 控制按键引脚

// GPIO对象定义
static gpio_t power_button_gpio;
static gpio_t control_button_gpio;

/*================================ 全局变量定义 ==============================*/
rtos_task_t key_task_handle = NULL;
rtos_task_t motor_task_handle = NULL;
rtos_task_t lcd_task_handle = NULL;

// 任务间通信标志
volatile uint8_t lcd_update_flag = 0;
volatile uint8_t motor_control_flag = 0;
volatile uint8_t system_power_state = 1; // 1=开机, 0=关机

// 按键对象定义
static Button power_button;
static Button control_button;

/*================================ 按键回调函数 ==============================*/
void power_button_callback(Button *btn)
{
    switch (btn->event) {
        case BTN_PRESS_DOWN:
            printf("Power button pressed\n");
            break;
            
        case BTN_PRESS_UP:
            printf("Power button released\n");
            // 切换系统电源状态
            set_system_power_state(!system_power_state);
            set_lcd_update_flag(); // 触发LCD更新
            break;
            
        case BTN_LONG_PRESS_START:
            printf("Power button long press - System shutdown\n");
            set_system_power_state(0);
            set_lcd_update_flag();
            break;
            
        default:
            break;
    }
}

void control_button_callback(Button *btn)
{
    switch (btn->event) {
        case BTN_PRESS_DOWN:
            printf("Control button pressed\n");
            break;
            
        case BTN_PRESS_UP:
            printf("Control button released\n");
            // 触发电机控制
            if (system_power_state) {
                set_motor_control_flag(1); // 正转
                set_lcd_update_flag();
            }
            break;
            
        case BTN_LONG_PRESS_START:
            printf("Control button long press - Reverse\n");
            if (system_power_state) {
                set_motor_control_flag(2); // 反转
                set_lcd_update_flag();
            }
            break;
            
        default:
            break;
    }
}

// GPIO读取函数示例 - 需要根据实际硬件修改
// 注意：按键库需要的是 uint8_t (*pin_level)(uint8_t button_id) 格式
uint8_t read_button_gpio_level(uint8_t button_id)
{
    // 根据button_id读取对应的GPIO状态
    switch (button_id) {
        case 0: // 电源按键
            return gpio_read(&power_button_gpio);
            
        case 1: // 控制按键
            return gpio_read(&control_button_gpio);
            
        default:
            return 1; // 默认返回未按下状态（高电平）
    }
}

/*================================ GPIO初始化函数 ==============================*/
static void button_gpio_init(void)
{
    // 初始化电源按键GPIO
    gpio_init(&power_button_gpio, POWER_BUTTON_PIN);
    gpio_dir(&power_button_gpio, PIN_INPUT);    // 设置为输入模式
    gpio_mode(&power_button_gpio, PullUp);      // 上拉电阻，按键按下为低电平
    
    // 初始化控制按键GPIO
    gpio_init(&control_button_gpio, CONTROL_BUTTON_PIN);
    gpio_dir(&control_button_gpio, PIN_INPUT);  // 设置为输入模式
    gpio_mode(&control_button_gpio, PullUp);    // 上拉电阻，按键按下为低电平
    
    printf("Button GPIO initialized: Power=%d, Control=%d\n", 
           POWER_BUTTON_PIN, CONTROL_BUTTON_PIN);
}

/*================================ 任务函数实现 ==============================*/

/**
 * @brief 按键扫描任务 - 5ms周期扫描
 */
void key_scan_task(void *param)
{
    UNUSED(param);
    
    printf("Key scan task started\n");
    
    // 初始化按键GPIO
    button_gpio_init();
    
    // 初始化按键对象
    // button_init(Button* handle, uint8_t(*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id)
    button_init(&power_button, read_button_gpio_level, 0, 0); // 低电平有效，按键ID=0
    button_init(&control_button, read_button_gpio_level, 0, 1); // 低电平有效，按键ID=1
    
    // 注册按键回调
    button_attach(&power_button, BTN_PRESS_DOWN, power_button_callback);
    button_attach(&power_button, BTN_PRESS_UP, power_button_callback);
    button_attach(&power_button, BTN_LONG_PRESS_START, power_button_callback);
    
    button_attach(&control_button, BTN_PRESS_DOWN, control_button_callback);
    button_attach(&control_button, BTN_PRESS_UP, control_button_callback);
    button_attach(&control_button, BTN_LONG_PRESS_START, control_button_callback);
    
    // 启动按键检测
    button_start(&power_button);
    button_start(&control_button);
    
    while (1) {
        // 调用按键扫描处理
        button_ticks();
        
        // 5ms周期扫描
        rtos_time_delay_ms(5);
    }
}

/**
 * @brief 电机控制任务
 */
void motor_control_task(void *param)
{
    UNUSED(param);
    
    printf("Motor control task started\n");
    
    while (1) {
        if (motor_control_flag && system_power_state) {
            switch (motor_control_flag) {
                case 1: // 正转
                    printf("Motor forward rotation\n");
                    stepper_motor_set_speed_profile(MOTOR_BASE, 900, 30);
                    // 这里可以添加具体的电机控制逻辑
                    break;
                    
                case 2: // 反转
                    printf("Motor reverse rotation\n");
                    stepper_motor_set_speed_profile(MOTOR_BASE, -900, 30);
                    // 这里可以添加具体的电机控制逻辑
                    break;
                    
                default:
                    break;
            }
            
            motor_control_flag = 0; // 清除标志
        }
        
        // 10ms周期检查
        rtos_time_delay_ms(10);
    }
}

/**
 * @brief LCD显示任务
 */
void lcd_display_task(void *param)
{
    UNUSED(param);
    static uint32_t last_update_time = 0;
    uint32_t current_time;
    
    printf("LCD display task started\n");
    
    // 初始化LCD
    DisplayLCD_Init();
    
    // 显示初始界面
    //LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0, LCD_H-1, BLACK);
    
    while (1) {
        current_time = rtos_time_get_current_system_time_ms();
        
        // 检查是否需要更新LCD或者定时刷新（每1秒强制刷新一次）
        if (lcd_update_flag || (current_time - last_update_time > 1000)) {
            
            // 根据系统状态显示不同内容
            if (!system_power_state) {

                 
                // 系统开机状态
                system_power_state=1;
             
            } else {
                 
                // 系统关机状态
                system_power_state=0;
            }
            
            last_update_time = current_time;
        }
        
        // 50ms周期检查，约20fps刷新率
        rtos_time_delay_ms(50);
    }
}

/*================================ 任务间通信函数 ==============================*/

void set_lcd_update_flag(void)
{
    lcd_update_flag = 1;
}

void set_motor_control_flag(uint8_t direction)
{
    motor_control_flag = direction;
}

void set_system_power_state(uint8_t state)
{
    system_power_state = state;
    printf("System power state changed to: %s\n", state ? "ON" : "OFF");
}

/*================================ 任务管理函数 ==============================*/

/**
 * @brief 初始化任务管理系统
 */
void task_manager_init(void)
{
    printf("Task manager initializing...\n");
    
    // 初始化相关硬件模块
    // 初始化步进电机（如果还没有在app_example中初始化）
    // stepper_motor_init();
    
    // 其他硬件初始化可以在这里添加
    
    printf("Task manager initialized successfully\n");
}

/**
 * @brief 启动所有任务
 */
void task_manager_start_all(void)
{
    int ret;
    
    printf("Starting all application tasks...\n");
    
    // 创建按键扫描任务 - 最高优先级
    ret = rtos_task_create(&key_task_handle,
                          "KeyScanTask",
                          key_scan_task,
                          NULL,
                          KEY_TASK_STACK_SIZE,
                          KEY_TASK_PRIORITY);
    if (ret != RTK_SUCCESS) {
        printf("Failed to create key scan task! Error: %d\n", ret);
        return;
    }
    printf("Key scan task created successfully\n");
    
    // 创建电机控制任务 - 中等优先级
    ret = rtos_task_create(&motor_task_handle,
                          "MotorTask",
                          motor_control_task,
                          NULL,
                          MOTOR_TASK_STACK_SIZE,
                          MOTOR_TASK_PRIORITY);
    if (ret != RTK_SUCCESS) {
        printf("Failed to create motor control task! Error: %d\n", ret);
        return;
    }
    printf("Motor control task created successfully\n");
    
    // 创建LCD显示任务 - 较低优先级
    ret = rtos_task_create(&lcd_task_handle,
                          "LCDTask",
                          lcd_display_task,
                          NULL,
                          LCD_TASK_STACK_SIZE,
                          LCD_TASK_PRIORITY);
    if (ret != RTK_SUCCESS) {
        printf("Failed to create LCD display task! Error: %d\n", ret);
        return;
    }
    printf("LCD display task created successfully\n");
    
    printf("All tasks started successfully!\n");
}

/**
 * @brief 停止所有任务
 */
void task_manager_stop_all(void)
{
    printf("Stopping all application tasks...\n");
    
    if (key_task_handle != NULL) {
        rtos_task_delete(key_task_handle);
        key_task_handle = NULL;
        printf("Key scan task stopped\n");
    }
    
    if (motor_task_handle != NULL) {
        rtos_task_delete(motor_task_handle);
        motor_task_handle = NULL;
        printf("Motor control task stopped\n");
    }
    
    if (lcd_task_handle != NULL) {
        rtos_task_delete(lcd_task_handle);
        lcd_task_handle = NULL;
        printf("LCD display task stopped\n");
    }
    
    printf("All tasks stopped\n");
}