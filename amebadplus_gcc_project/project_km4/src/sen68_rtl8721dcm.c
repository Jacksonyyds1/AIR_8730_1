#include "sen68_rtl8721dcm.h"

// I2C对象
static i2c_t sen68_i2c;
static bool i2c_initialized = false;

uint8_t sen68_i2c_init(void)
{
    if (!i2c_initialized) {
        // 根据你的硬件连接配置I2C引脚
        i2c_init(&sen68_i2c, I2C_SDA_PIN, I2C_SCL_PIN);
        i2c_frequency(&sen68_i2c, 50000); // 50kHz

        // 3. 添加延时确保I2C初始化完成
        rtos_time_delay_ms(10);

        i2c_initialized = true;
        printf("I2C initialized at 50KHZ\n");
    }
    return SEN68_OK;
}

uint8_t sen68_i2c_send_data(uint8_t addr, uint8_t *data, uint16_t len)
{
    if (!i2c_initialized) {
        return SEN68_ERROR;
    }
    
    int ret = i2c_write(&sen68_i2c, addr, (char*)data, len, 1);
    return (ret == len) ? SEN68_OK : SEN68_ERROR;
}

uint8_t sen68_i2c_receive_data(uint8_t addr, uint8_t *data, uint16_t len)
{
    if (!i2c_initialized) {
        return SEN68_ERROR;
    }
    
    int ret = i2c_read(&sen68_i2c, addr, (char*)data, len, 1);
    return (ret == len) ? SEN68_OK : SEN68_ERROR;
}

uint32_t sen68_get_tick(void)
{
    // 使用RTL8721DCM的系统时钟函数
    return rtos_time_get_current_system_time_ms();
}