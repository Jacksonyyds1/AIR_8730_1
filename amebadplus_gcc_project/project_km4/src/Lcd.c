// RTL8721DCM LCD移植代码 - lcd.c
#include "ameba_soc.h"
#include "os_wrapper.h"
#include "spi_api.h"
#include "spi_ex_api.h"
#include "gpio_api.h"
#include "ameba_gdma.h"
#include "Lcd.h"
#include "images.h"  


#ifndef DelayMs
#define DelayMs(ms) rtos_time_delay_ms(ms)
#endif


//static void LCD_Display_FullScreen_2(const uint16_t *flash_address);
// SPI对象
static spi_t spi_master;


#define DMA_BLOCK_SIZE 8192 // DMA传输块大小


//static void LCD_Display_FullScreen_2(uint16_t *flash_address);

// LCD控制引脚宏定义 - 需要根据实际硬件连接修改
#define LCD_RES_PIN         _PA_12
#define LCD_DC_PIN          _PA_22  


// SPI引脚定义 - 需要根据实际硬件连接修改
#define SPI_MOSI_PIN        _PA_23
#define SPI_MISO_PIN        _PA_20
#define SPI_SCLK_PIN        _PA_30
#define SPI_CS_PIN          _PA_21



// GPIO控制宏
#define LCD_RES_Set()       gpio_write(&lcd_res_gpio, 1)
#define LCD_RES_Clr()       gpio_write(&lcd_res_gpio, 0)
#define LCD_DC_Set()        gpio_write(&lcd_dc_gpio, 1)
#define LCD_DC_Clr()        gpio_write(&lcd_dc_gpio, 0)
#define LCD_CS_Set()        gpio_write(&lcd_cs_gpio, 1)
#define LCD_CS_Clr()        gpio_write(&lcd_cs_gpio, 0)
#define LCD_BL_Set()        gpio_write(&lcd_bl_gpio, 1)
#define LCD_BL_Clr()        gpio_write(&lcd_bl_gpio, 0)

// GPIO对象
static gpio_t lcd_res_gpio;
static gpio_t lcd_dc_gpio;
static gpio_t lcd_cs_gpio;
static gpio_t lcd_bl_gpio;

// 传输状态跟踪
typedef struct {
    volatile bool transfer_active;
    uint32_t transfer_count;
    uint32_t error_count;
} lcd_transfer_status_t;

static lcd_transfer_status_t transfer_status = {0};

// SPI传输完成回调
static void lcd_spi_dma_callback(uint32_t id, SpiIrq event)
{
    UNUSED(id);
    
    if (event == SpiTxIrq) {
        transfer_status.transfer_active = false;
        transfer_status.transfer_count++;
        // 传输完成后的处理（如果需要）
    } else {
        transfer_status.error_count++;
        printf("[ERROR] SPI DMA transfer error\n");
    }
}

// GPIO初始化
static void LCD_GPIO_Init(void)
{
    // 初始化复位引脚
    gpio_init(&lcd_res_gpio, LCD_RES_PIN);
    gpio_dir(&lcd_res_gpio, PIN_OUTPUT);
    gpio_mode(&lcd_res_gpio, PullUp);
    
    // 初始化数据/命令选择引脚
    gpio_init(&lcd_dc_gpio, LCD_DC_PIN);
    gpio_dir(&lcd_dc_gpio, PIN_OUTPUT);
    gpio_mode(&lcd_dc_gpio, PullUp);
    
    // 初始化片选引脚
    gpio_init(&lcd_cs_gpio, SPI_CS_PIN);
    gpio_dir(&lcd_cs_gpio, PIN_OUTPUT);
    gpio_mode(&lcd_cs_gpio, PullUp);
    
    // 初始化背光引脚
/*     gpio_init(&lcd_bl_gpio, LCD_BL_PIN);
    gpio_dir(&lcd_bl_gpio, PIN_OUTPUT);
    gpio_mode(&lcd_bl_gpio, PullUp); */
    
    // 初始化状态
    LCD_CS_Set();
    LCD_DC_Set();
    LCD_RES_Set();
    LCD_BL_Set();
}

// SPI初始化
static void LCD_SPI_Init(void)
{
    // 初始化SPI
    spi_init(&spi_master, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN, SPI_CS_PIN);
    spi_format(&spi_master, 8, 0, 0);  // 8位数据，模式0
    spi_frequency(&spi_master, 20000000);  // 20MHz
    printf("SPI initialized with MOSI: %d, MISO: %d, SCLK: %d, CS: %d\n",
           SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN, SPI_CS_PIN);
}
/*****************************************SPI_DMA  start*********************************************************************/
// DMA初始化
static void LCD_DMA_Init(void)
{
     spi_irq_hook(&spi_master, lcd_spi_dma_callback, 0);
}

static int LCD_SPI_DMA_Write(uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }
    if(spi_master.state & SPI_STATE_TX_BUSY){

        printf("[WARN] SPI busy, wating ...\n");
        return -2;
    }
    // 手动控制cs (保持LCD时序控制)
    LCD_CS_Clr();  
    int result = spi_master_write_stream_dma(&spi_master, (char*)data, length);

    if(result != HAL_OK){
        LCD_CS_Set(); 
        transfer_status.error_count++;
        printf("[ERROR] SPI DMA write failed, result: %d\n", result);
        return -3;
    }
    transfer_status.transfer_active = true;
    uint32_t timeout = 1000+ (length / 10000);

    while(transfer_status.transfer_active && (spi_master.state & SPI_STATE_TX_BUSY)&& timeout--){
        rtos_time_delay_ms(1);  // 等待传输完成
    }
    LCD_CS_Set();  // 传输完成后设置CS高电平

    if (timeout == 0){
        printf("[ERROR] Transfer timeout\n");
        return -4;
    }
    return 0;
}

// 批量写入16位图片数据
int LCD_Write_Buffer_DMA(const uint16_t *color_buffer, uint32_t pixel_count)
{
    if (color_buffer == NULL || pixel_count == 0) {
        return -1;
    }
    
    uint32_t chunk_pixels = DMA_BLOCK_SIZE / 2; 

    //分配固定大小的字节缓冲区
    uint8_t *byte_buffer = (uint8_t *)rtos_mem_malloc(DMA_BLOCK_SIZE);
    if (byte_buffer == NULL) {
        printf("Memory allocation failed for DMA buffer\n");
        return -1;
    }

    uint32_t remaining_pixels = pixel_count;
    uint32_t processed = 0;
    int result = 0;

    while (remaining_pixels > 0) {
        uint32_t current_pixels = (remaining_pixels > chunk_pixels) ? chunk_pixels : remaining_pixels;
        
        // 转换当前块的数据为8位字节序列
        for (uint32_t i = 0; i < current_pixels; i++) {
            uint16_t pixel = color_buffer[processed + i];
            byte_buffer[i * 2] = (pixel >> 8) & 0xFF;      // 高字节
            byte_buffer[i * 2 + 1] = pixel & 0xFF;         // 低字节
        }
        
        // 执行DMA传输
        result = LCD_SPI_DMA_Write(byte_buffer, current_pixels * 2);

        if(result !=0){
            printf("[ERROR] Chunk %lu transfer failed\n", processed / current_pixels);
        }
        remaining_pixels -= current_pixels;
        processed += current_pixels;
        
        // 显示进度（可选）
        if ((processed / chunk_pixels) % 10 == 0) {
            printf("Processed %lu/%lu pixels\n", processed, pixel_count);
        }
    }
    
    rtos_mem_free(byte_buffer);
    if(result == 0){
    printf("Color buffer DMA transfer completed: %lu pixels\n", pixel_count);
    }
    return result;
}
// 使用DMA区域填充颜色
void LCD_Fill_Area_DMA(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    // 参数验证
    if (x1 >= LCD_W || y1 >= LCD_H || x2 >= LCD_W || y2 >= LCD_H || x1 > x2 || y1 > y2) {
        printf("[ERROR] Invalid fill area parameters\n");
        return;
    }
    
    uint32_t total_pixels = (x2 - x1 + 1) * (y2 - y1 + 1);
    
    // 设置显示区域
    LCD_Address_Set(x1, y1, x2, y2);
    
    const uint32_t FILL_CHUNK = DMA_BLOCK_SIZE;
    uint8_t *fill_buffer = (uint8_t *)rtos_mem_malloc(FILL_CHUNK);
    
    if (fill_buffer == NULL) {
        printf("[ERROR] Fill buffer allocation failed\n");
        return;
    }
    
    // 预填充颜色数据
    uint32_t fill_pixels = FILL_CHUNK / 2;
    for (uint32_t i = 0; i < fill_pixels; i++) {
        fill_buffer[i * 2] = (color >> 8) & 0xFF;
        fill_buffer[i * 2 + 1] = color & 0xFF;
    }
    
    // 分块填充
    uint32_t remaining_pixels = total_pixels;
    uint32_t blocks = 0;
    
    while (remaining_pixels > 0) {
        uint32_t current_pixels = (remaining_pixels > fill_pixels) ? fill_pixels : remaining_pixels;
        
        
        if (LCD_SPI_DMA_Write(fill_buffer, current_pixels * 2) != 0) {
            printf("[ERROR] Fill block %lu failed\n", blocks);
            break;
        }
        
        remaining_pixels -= current_pixels;
        blocks++;
    }
    
    rtos_mem_free(fill_buffer);
    printf("[INFO] Fill completed: %lu blocks\n", blocks);
}

// 显示图像缓冲区（使用DMA）
void LCD_Display_Image_DMA(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *image_data)
{
    if (image_data == NULL || x + width > LCD_W || y + height > LCD_H) {
        return;
    }
    
    // 设置显示区域
    LCD_Address_Set(x, y, x + width - 1, y + height - 1);
    
    // 使用新的官方API传输
    uint32_t total_pixels = width * height;
    if (LCD_Write_Buffer_DMA(image_data, total_pixels) != 0) {
        printf("[ERROR] Image display failed\n");
    } else {
        printf("[INFO] Image displayed: %dx%d at (%d,%d)\n", width, height, x, y);
    }
}

/*****************************************SPI_DMA  end*********************************************************************/
// 写入单个数据
void WriteData(uint8_t dat)
{
    LCD_CS_Clr();
    spi_master_write(&spi_master, dat);
    LCD_CS_Set();
}

// 写入命令
void WriteComm(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();  // 写命令
    spi_master_write(&spi_master, dat);
    LCD_CS_Set();
    LCD_DC_Set();  // 写数据
}

// LCD写入16位数据
void LCD_WR_DATA(uint16_t dat)
{
    LCD_CS_Clr();
    spi_master_write(&spi_master, dat >> 8);
    spi_master_write(&spi_master, dat & 0xFF);
    LCD_CS_Set();
}

// LCD写入寄存器
void LCD_WR_REG(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();  // 写命令
    spi_master_write(&spi_master, dat);
    LCD_CS_Set();
    LCD_DC_Set();  // 写数据
}

// 设置显示地址
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    printf("Settint address:(%d, %d) to (%d, %d)\n", x1, y1, x2, y2);

    LCD_WR_REG(0x2a);  // 列地址设置
    LCD_WR_DATA(x1+ST7789V2_X_OFFSET);// 
    LCD_WR_DATA(x2+ST7789V2_X_OFFSET);// 
    LCD_WR_REG(0x2b);  // 行地址设置
    LCD_WR_DATA(y1+ST7789V2_Y_OFFSET);
    LCD_WR_DATA(y2+ST7789V2_Y_OFFSET);
    LCD_WR_REG(0x2c);  // 储存器写
}

// 简化的填充函数（不使用DMA）
void LCD_Fill_FixedColor_Simple(uint8_t xsta, uint8_t xend, uint8_t ysta, uint8_t yend, uint16_t color)
{
    uint32_t total_pixels;
    uint32_t i;
    
    if (xsta >= LCD_W || ysta >= LCD_H || xend > LCD_W || yend > LCD_H || xsta >= xend || ysta >= yend)
        return;
    
    LCD_Address_Set(xsta, ysta, xend, yend);
    
    total_pixels = (xend - xsta + 1) * (yend - ysta + 1);
    
    LCD_CS_Clr();
    for (i = 0; i < total_pixels; i++) {
        spi_master_write(&spi_master, color >> 8);
        spi_master_write(&spi_master, color & 0xFF);
    }
    LCD_CS_Set();
}

// LCD初始化
void DisplayLCD_Init(void)
{
    // 初始化GPIO
    LCD_GPIO_Init();
    
    spi_master.spi_idx = MBED_SPI1;

    // 初始化SPI
    LCD_SPI_Init();
    
    // 初始化DMA
    LCD_DMA_Init();
    
    // 背光开启
    //LCD_BL_Set();
    
    // 复位时序
    LCD_RES_Set();
    DelayMs(100);
    LCD_RES_Clr();
    DelayMs(100);
    LCD_RES_Set();
    DelayMs(120);
    
       // 完整的ST7789V2初始化序列
    WriteComm(0x11);     // Sleep out
    DelayMs(120);
    
    WriteComm(0x36);     // Memory Access Control
    WriteData(0x00);     // 或 0xC8 如果需要BGR格式
    
    WriteComm(0x3A);     // Pixel Format Set
    WriteData(0x05);     // 16-bit RGB565
    
    WriteComm(0xB2);     // Porch Setting
    WriteData(0x0C);
    WriteData(0x0C);
    WriteData(0x00);
    WriteData(0x33);
    WriteData(0x33);
    
    WriteComm(0xB7);     // Gate Control
    WriteData(0x71);
    
    WriteComm(0xBB);     // VCOM Setting
    WriteData(0x3B);
    
    WriteComm(0xC0);     // LCM Control
    WriteData(0x2C);
    
    WriteComm(0xC2);     // VDV and VRH Command Enable
    WriteData(0x01);
    
    WriteComm(0xC3);     // VRH Set
    WriteData(0x13);
    
    WriteComm(0xC4);     // VDV Set
    WriteData(0x20);
    
    WriteComm(0xC6);     // Frame Rate Control
    WriteData(0x0F);
    
    WriteComm(0xD0);     // Power Control 1
    WriteData(0xA4);
    WriteData(0xA1);
    
    WriteComm(0xD6);     
    WriteData(0xA1);
    
    // 正伽马校正
    WriteComm(0xE0);
    WriteData(0xD0); 
    WriteData(0x08);
    WriteData(0x0A); 
    WriteData(0x0D);
    WriteData(0x0B);
    WriteData(0x07); 
    WriteData(0x21); 
    WriteData(0x33);
    WriteData(0x39);
    WriteData(0x39); 
    WriteData(0x16);
    WriteData(0x16);
    WriteData(0x1F);
    WriteData(0x3C);
    
    // 负伽马校正
    WriteComm(0xE1);
    WriteData(0xD0); 
    WriteData(0x00); 
    WriteData(0x03);
    WriteData(0x01);
    WriteData(0x00); 
    WriteData(0x10); 
    WriteData(0x21); 
    WriteData(0x32);
    WriteData(0x38);
    WriteData(0x16); 
    WriteData(0x14);
    WriteData(0x14);
    WriteData(0x20);
    WriteData(0x3D);
    
    WriteComm(0x21);     // Display Inversion On
    DelayMs(120);
    
    WriteComm(0x29);     // Display On
    DelayMs(120);
    
    // 填充黑色
    //LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0,LCD_H-1, WHITE);
    printf("LCD initialized successfully\n");

}

// 显示全屏图片
void LCD_Display_FullScreen(uint16_t *flash_address)
{
    uint32_t total_pixels = LCD_W * LCD_H;
    uint32_t i;
    
    LCD_Address_Set(0, 0, LCD_W-1, LCD_H-1);
    
    LCD_CS_Clr();
    for (i = 0; i < total_pixels; i++) {
        spi_master_write(&spi_master, flash_address[i] >> 8);
        spi_master_write(&spi_master, flash_address[i] & 0xFF);
    }
    LCD_CS_Set();
}

// 背光控制
void LCD_BacklightOnOff(uint8_t onoff)
{
    if (onoff) {
        LCD_BL_Set();
    } else {
        LCD_BL_Clr();
    }
}