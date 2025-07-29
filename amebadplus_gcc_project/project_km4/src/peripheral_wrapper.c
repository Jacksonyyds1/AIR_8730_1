#include "peripheral_wrapper.h"
#include "PeripheralNames.h"  // 只在C文件中包含

// 内部映射函数
static UARTName get_uart_name(peripheral_id_t uart_id) {
    switch(uart_id) {
        case PERIPHERAL_UART_0: return UART_0;
        case PERIPHERAL_UART_1: return UART_1;
        case PERIPHERAL_UART_2: return UART_2;
        default: return UART_0;
    }
}

static SPIName get_spi_name(peripheral_id_t spi_id) {
    switch(spi_id) {
        case PERIPHERAL_SPI_0: return SPI_0;
        case PERIPHERAL_SPI_1: return SPI_1;
        default: return SPI_0;
    }
}

// 实现接口函数
int peripheral_init_uart(peripheral_id_t uart_id) {
    UARTName uart_name = get_uart_name(uart_id);
    // 调用原始的UART初始化函数
    return uart_init_with_name(uart_name);
}

int peripheral_init_spi(peripheral_id_t spi_id) {
    SPIName spi_name = get_spi_name(spi_id);
    // 调用原始的SPI初始化函数
    return spi_init_with_name(spi_name);
}