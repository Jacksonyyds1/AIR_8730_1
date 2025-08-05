#ifndef __SEN68_RTL8721DCM_H
#define __SEN68_RTL8721DCM_H

// RTL8721DCM平台头文件
#include "ameba_soc.h"
#include "os_wrapper.h"
#include "i2c_api.h"
#include "gpio_api.h"
#include "IO_driver.h"  // 使用你现有的IO驱动

// 保持原有的SEN68协议定义
#include "sen68.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_SDA_PIN _PB_1
#define I2C_SCL_PIN _PB_0

// 平台适配函数
uint8_t sen68_i2c_init(void);
uint8_t sen68_i2c_send_data(uint8_t addr, uint8_t *data, uint16_t len);
uint8_t sen68_i2c_receive_data(uint8_t addr, uint8_t *data, uint16_t len);
uint32_t sen68_get_tick(void);

#ifdef __cplusplus
}
#endif

#endif