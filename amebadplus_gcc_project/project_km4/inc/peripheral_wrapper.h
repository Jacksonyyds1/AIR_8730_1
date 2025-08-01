#ifndef PERIPHERAL_WRAPPER_H
#define PERIPHERAL_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// 定义自己的外设ID枚举（纯整数）
typedef enum {
    PERIPHERAL_UART_0 = 0,
    PERIPHERAL_UART_1 = 1,
    PERIPHERAL_UART_2 = 2,
    PERIPHERAL_SPI_0 = 0,
    PERIPHERAL_SPI_1 = 1,
    PERIPHERAL_I2C_0 = 0,
    PERIPHERAL_I2C_1 = 1,
} peripheral_id_t;

// 提供C接口函数
int peripheral_init_uart(peripheral_id_t uart_id);
int peripheral_init_spi(peripheral_id_t spi_id);
int peripheral_init_i2c(peripheral_id_t i2c_id);

// 如果需要声明这些函数，可以添加以下声明
#ifdef USE_CUSTOM_INIT_FUNCTIONS
// 声明自定义的初始化函数
int uart_init_with_name(UARTName uart_name);
int spi_init_with_name(SPIName spi_name);
#endif

#ifdef __cplusplus
}
#endif

#endif