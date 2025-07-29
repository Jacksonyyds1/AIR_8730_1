
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

#ifdef __cplusplus
}
#endif

#endif