#ifndef __IO_DRIVER_H
#define __IO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

// RealTek平台GPIO头文件
#include "ameba_soc.h"
#include "gpio_api.h"
#include "PinNames.h"

// GPIO对象声明
extern gpio_t wifi_rst_gpio;
extern gpio_t ionizer_gpio;
extern gpio_t uv_light_gpio;
extern gpio_t scd40_power_gpio;
extern gpio_t sen68_power_gpio;
extern gpio_t encoder_base_power_gpio;
extern gpio_t encoder_base_gpio;
extern gpio_t encoder_neck_power_gpio;
extern gpio_t encoder_neck_gpio;

#ifdef TEST_PIN 
extern gpio_t test_gpio;
#endif
// 测试引脚定义(可选)
 //#define TEST_PIN       PA_10
// #define test_pin_on()  gpio_write(&test_gpio, 1)
// #define test_pin_off() gpio_write(&test_gpio, 0)

// WIFI控制引脚
#define WIFI_RST_PIN    PA_1
#define WIFI_on()       gpio_write(&wifi_rst_gpio, 1)
#define WIFI_off()      gpio_write(&wifi_rst_gpio, 0)

/* // 离子发生器控制引脚
#define IONIZER_PIN     PB_10
#define ionizer_on()         gpio_write(&ionizer_gpio, 1)
#define ionizer_off()        gpio_write(&ionizer_gpio, 0)
#define ionizer_get_status() gpio_read(&ionizer_gpio)

// UV灯控制引脚
#define UV_LIGHT_PIN    PC_11
#define uv_light_on()         gpio_write(&uv_light_gpio, 1)
#define uv_light_off()        gpio_write(&uv_light_gpio, 0)
#define uv_light_get_status() gpio_read(&uv_light_gpio)

// SCD40传感器电源控制引脚
#define SCD40_POWER_PIN PC_14
#define scd40_power_on()  gpio_write(&scd40_power_gpio, 1)
#define scd40_power_off() gpio_write(&scd40_power_gpio, 0) */

// SEN68传感器电源控制引脚
#define SEN68_POWER_PIN  PB_10
#define sen68_power_on()  gpio_write(&sen68_power_gpio, 1)
#define sen68_power_off() gpio_write(&sen68_power_gpio, 0)

// 底座编码器相关引脚
#define ENCODER_BASE_POWER_PIN PB_7
#define ENCODER_BASE_PIN       PB_3
#define encoder_base_power_on()  gpio_write(&encoder_base_power_gpio, 1)
#define encoder_base_power_off() gpio_write(&encoder_base_power_gpio, 0)
#define encoder_base_read()      gpio_read(&encoder_base_gpio)

// 颈部编码器相关引脚
#define ENCODER_NECK_POWER_PIN PA_12
#define ENCODER_NECK_PIN       PA_11
#define encoder_neck_power_on()  gpio_write(&encoder_neck_power_gpio, 1)
#define encoder_neck_power_off() gpio_write(&encoder_neck_power_gpio, 0)
#define encoder_neck_read()      gpio_read(&encoder_neck_gpio)



/**
 * @brief 初始化所有IO引脚
 */
void io_driver_init(void);

/**
 * @brief 反初始化所有IO引脚
 */
void io_driver_deinit(void);

/**
 * @brief 获取GPIO引脚状态
 * @param gpio_obj GPIO对象指针
 * @return 引脚状态 (0或1)
 */
int io_driver_get_pin_status(gpio_t *gpio_obj);

/**
 * @brief 设置GPIO引脚状态
 * @param gpio_obj GPIO对象指针
 * @param value 要设置的值 (0或1)
 */
void io_driver_set_pin_status(gpio_t *gpio_obj, int value);

#ifdef __cplusplus
}
#endif

#endif  // __IO_DRIVER_H