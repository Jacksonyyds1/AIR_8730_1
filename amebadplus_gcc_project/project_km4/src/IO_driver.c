#include "IO_driver.h"
#include <stdio.h>

// GPIO对象定义
gpio_t wifi_rst_gpio;
gpio_t ionizer_gpio;
gpio_t uv_light_gpio;
gpio_t scd40_power_gpio;
gpio_t sen68_power_gpio;
gpio_t encoder_base_power_gpio;
gpio_t encoder_base_gpio;
gpio_t encoder_neck_power_gpio;
gpio_t encoder_neck_gpio;

#ifdef TEST_PIN
gpio_t test_gpio;
#endif

void io_driver_init(void)
{
    printf("IO Driver: Initializing GPIO pins...\n");

    // 初始化WIFI复位引脚 (输出)
    gpio_init(&wifi_rst_gpio, WIFI_RST_PIN);
    gpio_dir(&wifi_rst_gpio, PIN_OUTPUT);
    gpio_mode(&wifi_rst_gpio, PullNone);
    
   /*  // 初始化离子发生器控制引脚 (输出)
    gpio_init(&ionizer_gpio, IONIZER_PIN);
    gpio_dir(&ionizer_gpio, PIN_OUTPUT);
    gpio_mode(&ionizer_gpio, PullNone);
    
    // 初始化UV灯控制引脚 (输出)
    gpio_init(&uv_light_gpio, UV_LIGHT_PIN);
    gpio_dir(&uv_light_gpio, PIN_OUTPUT);
    gpio_mode(&uv_light_gpio, PullNone);
    
    // 初始化SCD40电源控制引脚 (输出)
    gpio_init(&scd40_power_gpio, SCD40_POWER_PIN);
    gpio_dir(&scd40_power_gpio, PIN_OUTPUT);
    gpio_mode(&scd40_power_gpio, PullNone); */
    
    // 初始化SEN68电源控制引脚 (输出)
    gpio_init(&sen68_power_gpio, SEN68_POWER_PIN);
    gpio_dir(&sen68_power_gpio, PIN_OUTPUT);
    gpio_mode(&sen68_power_gpio, PullNone);
    
    // 初始化底座编码器电源控制引脚 (输出)
    gpio_init(&encoder_base_power_gpio, ENCODER_BASE_POWER_PIN);
    gpio_dir(&encoder_base_power_gpio, PIN_OUTPUT);
    gpio_mode(&encoder_base_power_gpio, PullNone);
    
    // 初始化底座编码器信号引脚 (输入)
    gpio_init(&encoder_base_gpio, ENCODER_BASE_PIN);
    gpio_dir(&encoder_base_gpio, PIN_INPUT);
    gpio_mode(&encoder_base_gpio, PullUp);  // 使用上拉
    
    // 初始化颈部编码器电源控制引脚 (输出)
    gpio_init(&encoder_neck_power_gpio, ENCODER_NECK_POWER_PIN);
    gpio_dir(&encoder_neck_power_gpio, PIN_OUTPUT);
    gpio_mode(&encoder_neck_power_gpio, PullNone);
    
    // 初始化颈部编码器信号引脚 (输入)
    gpio_init(&encoder_neck_gpio, ENCODER_NECK_PIN);
    gpio_dir(&encoder_neck_gpio, PIN_INPUT);
    gpio_mode(&encoder_neck_gpio, PullUp);  // 使用上拉

#ifdef TEST_PIN
    // 初始化测试引脚 (输出)
    gpio_init(&test_gpio, TEST_PIN);
    gpio_dir(&test_gpio, PIN_OUTPUT);
    gpio_mode(&test_gpio, PullNone);
#endif

    // 设置所有输出引脚的初始状态
    WIFI_off();                    // 确保WIFI初始关闭
   /*  ionizer_off();                 // 确保离子发生器初始关闭
    uv_light_off();               // 确保UV灯初始关闭
    scd40_power_off();            // 确保SCD40电源初始关闭 */
    sen68_power_off();            // 确保SEN68电源初始关闭
    encoder_base_power_off();     // 确保底座编码器电源初始关闭
    encoder_neck_power_off();     // 确保颈部编码器电源初始关闭

#ifdef TEST_PIN
    test_pin_off();               // 确保测试引脚初始关闭
#endif

    printf("IO Driver: GPIO initialization completed\n");
}

void io_driver_deinit(void)
{
    printf("IO Driver: Deinitializing GPIO pins...\n");
    
    // 关闭所有输出设备
    WIFI_off();
   /*  ionizer_off();
    uv_light_off();
    scd40_power_off(); */
    sen68_power_off();
    encoder_base_power_off();
    encoder_neck_power_off();

#ifdef TEST_PIN
    test_pin_off();
#endif

    // RealTek GPIO API通常不需要显式的去初始化
    // 如果需要，可以在这里添加相应的去初始化代码
    
    printf("IO Driver: GPIO deinitialization completed\n");
}

int io_driver_get_pin_status(gpio_t *gpio_obj)
{
    if (gpio_obj == NULL) {
        printf("IO Driver: Error - NULL GPIO object\n");
        return -1;
    }
    
    return gpio_read(gpio_obj);
}

void io_driver_set_pin_status(gpio_t *gpio_obj, int value)
{
    if (gpio_obj == NULL) {
        printf("IO Driver: Error - NULL GPIO object\n");
        return;
    }
    
    gpio_write(gpio_obj, value ? 1 : 0);
}

// 调试和状态函数
void io_driver_print_status(void)
{
    printf("=== IO Driver Status ===\n");
    printf("WIFI RST:            %d\n", gpio_read(&wifi_rst_gpio));
    printf("Ionizer:             %d\n", gpio_read(&ionizer_gpio));
    printf("UV Light:            %d\n", gpio_read(&uv_light_gpio));
    printf("SCD40 Power:         %d\n", gpio_read(&scd40_power_gpio));
    printf("SEN68 Power:         %d\n", gpio_read(&sen68_power_gpio));
    printf("Encoder Base Power:  %d\n", gpio_read(&encoder_base_power_gpio));
    printf("Encoder Base Signal: %d\n", gpio_read(&encoder_base_gpio));
    printf("Encoder Neck Power:  %d\n", gpio_read(&encoder_neck_power_gpio));
    printf("Encoder Neck Signal: %d\n", gpio_read(&encoder_neck_gpio));
    
#ifdef TEST_PIN
    printf("Test Pin:            %d\n", gpio_read(&test_gpio));
#endif
    printf("========================\n");
}