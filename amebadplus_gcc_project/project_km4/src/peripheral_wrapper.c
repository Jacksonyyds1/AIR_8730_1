#include "peripheral_wrapper.h"
#include "PeripheralNames.h"  // 只在C文件中包含
// 添加必要的SDK头文件
#include "serial_api.h"       // UART相关API
#include "spi_api.h"          // SPI相关API

// 内部映射函数 - 将外设ID映射到具体的Pin引脚
static PinName get_uart_tx_pin(peripheral_id_t uart_id) {
    switch(uart_id) {
        case PERIPHERAL_UART_0: return _PA_7;  // UART0 TX引脚
        case PERIPHERAL_UART_1: return _PA_14; // UART1 TX引脚
        case PERIPHERAL_UART_2: return _PA_28; // UART2 TX引脚
        default: return _PA_7;
    }
}

static PinName get_uart_rx_pin(peripheral_id_t uart_id) {
    switch(uart_id) {
        case PERIPHERAL_UART_0: return _PA_8;  // UART0 RX引脚
        case PERIPHERAL_UART_1: return _PA_15; // UART1 RX引脚
        case PERIPHERAL_UART_2: return _PA_29; // UART2 RX引脚
        default: return _PA_8;
    }
}

static void get_spi_pins(peripheral_id_t spi_id, PinName *mosi, PinName *miso, PinName *sclk, PinName *cs) {
    switch(spi_id) {
        case PERIPHERAL_SPI_0:
            *mosi = _PA_23;  // SPI0 MOSI引脚
            *miso = _PA_20;  // SPI0 MISO引脚
            *sclk = _PA_30;  // SPI0 SCLK引脚
            *cs   = _PA_21;  // SPI0 CS引脚
            break;
        case PERIPHERAL_SPI_1:
            *mosi = _PA_12;  // SPI1 MOSI引脚
            *miso = _PA_13;  // SPI1 MISO引脚
            *sclk = _PA_11;  // SPI1 SCLK引脚
            *cs   = _PA_10;  // SPI1 CS引脚
            break;
        default:
            *mosi = _PA_23;
            *miso = _PA_20;
            *sclk = _PA_30;
            *cs   = _PA_21;
            break;
    }
}

// 实现接口函数
int peripheral_init_uart(peripheral_id_t uart_id) {
    PinName tx_pin = get_uart_tx_pin(uart_id);
    PinName rx_pin = get_uart_rx_pin(uart_id);
    
    // 使用实际的mbed UART API - 参数是TX和RX引脚
    serial_t serial_obj;
    serial_init(&serial_obj, tx_pin, rx_pin);  // 正确的参数：TX引脚, RX引脚
    serial_baud(&serial_obj, 115200);          // 设置波特率
    serial_format(&serial_obj, 8, ParityNone, 1);  // 8N1格式
    
    return 0;  // 成功返回0
}

int peripheral_init_spi(peripheral_id_t spi_id) {
    PinName mosi, miso, sclk, cs;
    get_spi_pins(spi_id, &mosi, &miso, &sclk, &cs);
    
    // 使用实际的mbed SPI API - 参数是具体的引脚
    spi_t spi_obj;
    spi_init(&spi_obj, mosi, miso, sclk, cs);   // 正确的参数：MOSI, MISO, SCLK, CS引脚
    spi_format(&spi_obj, 8, 0, 0);             // 8位数据，模式0
    spi_frequency(&spi_obj, 1000000);          // 1MHz频率
    
    return 0;  // 成功返回0
}

// 方案2：如果你确实需要这些函数，可以自己实现
#ifdef USE_CUSTOM_INIT_FUNCTIONS
// 如果确实需要uart_init_with_name函数，可以这样实现：
int uart_init_with_name(UARTName uart_name) {
    serial_t serial_obj;
    serial_init(&serial_obj, uart_name, NC);
    serial_baud(&serial_obj, 115200);
    serial_format(&serial_obj, 8, ParityNone, 1);
    return 0;
}

// 如果确实需要spi_init_with_name函数，可以这样实现：
int spi_init_with_name(SPIName spi_name) {
    spi_t spi_obj;
    spi_init(&spi_obj, NC, NC, NC, NC);
    spi_format(&spi_obj, 8, 0, 0);
    spi_frequency(&spi_obj, 1000000);
    return 0;
}
#endif