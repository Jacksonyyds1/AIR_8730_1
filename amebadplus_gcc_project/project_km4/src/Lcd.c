// RTL8721DCM LCD移植代码 - lcd.c
#include "ameba_soc.h"
#include "os_wrapper.h"
#include "spi_api.h"
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
static GDMA_InitTypeDef GDMA_InitStruct;

static SPI_TypeDef *spi_device = NULL;  // SPI设备指针

#define DMA_BLOCK_SIZE 4096 // DMA传输块大小


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

// DMA完成标志
static volatile u32 DMA_Complete_Flag = 0;



// 增强的DMA回调函数
static u32 LCD_DMA_Callback(void *para)
{
    UNUSED(para);
    
    u32 IsrTypeMap = GDMA_ClearINT(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum);
    
    if (IsrTypeMap & TransferType) {
        DMA_Complete_Flag = 1;
        // 可以在这里添加传输完成后的处理
    }
    
    if (IsrTypeMap & ErrType) {
        printf("DMA transfer error occurred\n");
        DMA_Complete_Flag = 2;  // 错误标志
    }
    
    return 0;
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
    spi_frequency(&spi_master, 10000000);  // 10MHz
    printf("SPI initialized with MOSI: %d, MISO: %d, SCLK: %d, CS: %d\n",
           SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN, SPI_CS_PIN);
}
/*****************************************SPI_DMA  start*********************************************************************/
// DMA初始化
static void LCD_DMA_Init(void)
{
     // 获取SPI设备指针
     uint8_t spi_index = 1;
    spi_device = SPI_DEV_TABLE[spi_index].SPIx;
    // 初始化 GDMA 结构体
    GDMA_StructInit(&GDMA_InitStruct);
    
    // 设置 DMA 基本参数 - 针对SPI传输优化
    GDMA_InitStruct.GDMA_Index = 0;
    GDMA_InitStruct.GDMA_DIR = TTFCMemToPeri;
    GDMA_InitStruct.GDMA_SrcMsize = MsizeFour;        // 参考示例代码
    GDMA_InitStruct.GDMA_SrcDataWidth = TrWidthOneByte;
    GDMA_InitStruct.GDMA_DstMsize = MsizeFour;         // 参考示例代码
    GDMA_InitStruct.GDMA_DstDataWidth = TrWidthOneByte;
    GDMA_InitStruct.GDMA_SrcInc = IncType;
    GDMA_InitStruct.GDMA_DstInc = NoChange;
    GDMA_InitStruct.GDMA_IsrType = (TransferType | ErrType);
    GDMA_InitStruct.GDMA_ReloadSrc = 0;
    GDMA_InitStruct.GDMA_ReloadDst = 0;
    
    // 设置正确的SPI握手接口
    GDMA_InitStruct.GDMA_SrcHandshakeInterface = 0;
    GDMA_InitStruct.GDMA_DstHandshakeInterface = SPI_DEV_TABLE[spi_index].Tx_HandshakeInterface; // 根据实际SPI调整
    
    #ifndef CONFIG_AMEBAD
    GDMA_InitStruct.GDMA_DstAddr = (u32)&SPI_DEV_TABLE[spi_index].SPIx->SPI_DRx;
    #else
    GDMA_InitStruct.GDMA_DstAddr = (u32)&SPI_DEV_TABLE[spi_index].SPIx->DR;
    #endif

    // 申请DMA通道
    GDMA_InitStruct.GDMA_ChNum = GDMA_ChnlAlloc(GDMA_InitStruct.GDMA_Index, 
                                               LCD_DMA_Callback, 
                                               0, 
                                               6);
    
    if (GDMA_InitStruct.GDMA_ChNum == 0xFF) {
        printf("DMA channel allocation failed\n");
        return;
    }
    
    printf("SPI DMA channel %d allocated successfully\n", GDMA_InitStruct.GDMA_ChNum);
}

// SPI DMA批量传输数据,最大传输长度为4096字节
static int LCD_SPI_DMA_Write(uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }
    
    // 清除DMA完成标志
    DMA_Complete_Flag = 0;
    
    // 配置DMA传输参数
    GDMA_InitStruct.GDMA_SrcAddr = (u32)data;
    GDMA_InitStruct.GDMA_BlockSize = length;

    // 数据缓存清理
    DCache_CleanInvalidate((u32)data, length);
    
    // 初始化DMA传输
    GDMA_Init(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, &GDMA_InitStruct);
    
    // 启动片选
    LCD_CS_Clr();

    // 启用SPI DMA
    SSI_SetDmaEnable(spi_device, ENABLE, SPI_BIT_TDMAE);
    
    // 启动DMA传输
    GDMA_Cmd(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, ENABLE);
    
    // 等待DMA完成
    uint32_t timeout = 1000;  // 1秒超时
    while (DMA_Complete_Flag == 0 && timeout > 0) {
        rtos_time_delay_ms(1);
        timeout--;
    }
    
    // 停止DMA
    GDMA_Cmd(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, DISABLE);

    // 禁用SPI DMA
    SSI_SetDmaEnable(spi_device, DISABLE, SPI_BIT_TDMAE);
    
    // 释放片选
    LCD_CS_Set();
    
    if (timeout == 0) {
        printf("DMA transfer timeout\n");
        return -1;
    }
    
    return 0;
}

// 批量写入16位图片数据
int LCD_Write_Color_Buffer_DMA(const uint16_t *color_buffer, uint32_t pixel_count)
{
    if (color_buffer == NULL || pixel_count == 0) {
        return -1;
    }
    
    uint32_t chunk_pixels = DMA_BLOCK_SIZE / 2; // 每块2048像素

    //分配固定大小的字节缓冲区
    uint8_t *byte_buffer = (uint8_t *)rtos_mem_malloc(DMA_BLOCK_SIZE);
    if (byte_buffer == NULL) {
        printf("Memory allocation failed for DMA buffer\n");
        return -1;
    }

    uint32_t remaining_pixels = pixel_count;
    uint32_t processed = 0;

    while (remaining_pixels > 0) {
        uint32_t current_pixels = (remaining_pixels > chunk_pixels) ? chunk_pixels : remaining_pixels;
        
        // 转换当前块的数据为字节序列
        for (uint32_t i = 0; i < current_pixels; i++) {
            uint16_t pixel = color_buffer[processed + i];
            byte_buffer[i * 2] = (pixel >> 8) & 0xFF;      // 高字节
            byte_buffer[i * 2 + 1] = pixel & 0xFF;         // 低字节
        }
        
        // 执行DMA传输
        if (LCD_SPI_DMA_Write(byte_buffer, current_pixels * 2) != 0) {
            printf("DMA transfer failed at chunk %lu\n", processed / chunk_pixels);
            rtos_mem_free(byte_buffer);
            return -1;
        }
        
        remaining_pixels -= current_pixels;
        processed += current_pixels;
        
        // 显示进度（可选）
        if ((processed / chunk_pixels) % 10 == 0) {
            printf("Processed %lu/%lu pixels\n", processed, pixel_count);
        }
    }
    
    rtos_mem_free(byte_buffer);
    printf("Color buffer DMA transfer completed: %lu pixels\n", pixel_count);
    return 0;
}
// 使用DMA区域填充颜色
void LCD_Fill_Area_DMA(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (x1 >= LCD_W || y1 >= LCD_H || x2 >= LCD_W || y2 >= LCD_H || x1 > x2 || y1 > y2) {
        return;
    }
    
    uint32_t total_pixels = (x2 - x1 + 1) * (y2 - y1 + 1);
    uint32_t total_bytes = total_pixels * 2; // 每个像素16位

    printf("Total pixels: %lu, Total bytes: %lu\n", total_pixels, total_bytes);
    
    // 设置显示区域
    LCD_Address_Set(x1, y1, x2, y2);
    
    
    // 分配合理大小的缓冲区 (2048像素 = 4096字节)
    uint32_t chunk_pixels = DMA_BLOCK_SIZE / 2;  // 每块2048像素
    uint8_t *byte_buffer = (uint8_t *)rtos_mem_malloc(DMA_BLOCK_SIZE);
    
    if (byte_buffer == NULL) {
        printf("Memory allocation failed for DMA buffer\n");
        return;
    }
    
    // 填充颜色数据
    for (uint32_t i = 0; i < chunk_pixels; i++) {
        byte_buffer[i * 2] = (color >> 8) & 0xFF;      // 高字节
        byte_buffer[i * 2 + 1] = color & 0xFF;        // 低字节
    }

        // 分块传输
        uint32_t remaining_pixels = total_pixels;
        uint32_t transferred = 0;
    
        while (remaining_pixels > 0) {
        uint32_t current_pixels = (remaining_pixels > chunk_pixels) ? chunk_pixels : remaining_pixels;
        uint32_t current_bytes = current_pixels * 2;
        
        // 如果是最后一块且不满，需要重新填充缓冲区
        if (current_pixels < chunk_pixels) {
            for (uint32_t i = 0; i < current_pixels; i++) {
                byte_buffer[i * 2] = (color >> 8) & 0xFF;
                byte_buffer[i * 2 + 1] = color & 0xFF;
            }
        }
        
        // 执行DMA传输
        if (LCD_SPI_DMA_Write(byte_buffer, current_bytes) != 0) {
            printf("DMA transfer failed at block %lu\n", transferred);
            break;
        }
        
        remaining_pixels -= current_pixels;
        transferred++;
        
        // 显示进度
        if (transferred % 10 == 0) {
            printf("Transferred %lu blocks, remaining pixels: %lu\n", transferred, remaining_pixels);
        }
    }
    // 释放内存
    rtos_mem_free(byte_buffer);
    printf("DMA fill completed,total blocks:%lu\n", transferred);
}

// 显示图像缓冲区（使用DMA）
void LCD_Display_Image_DMA(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *image_data)
{
    if (image_data == NULL || x + width > LCD_W || y + height > LCD_H) {
        return;
    }
    
    // 设置显示区域
    LCD_Address_Set(x, y, x + width - 1, y + height - 1);
    
    // 使用DMA传输图像数据
    uint32_t total_pixels = width * height;
    if (LCD_Write_Color_Buffer_DMA(image_data, total_pixels) != 0) {
        printf("DMA image transfer failed\n");
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
    DelayMs(1);
    LCD_RES_Clr();
    DelayMs(10);
    LCD_RES_Set();
    DelayMs(120);
    
    // LCD初始化序列
    WriteComm(0x11);     // Sleep out
    DelayMs(120);
    
    WriteComm(0xB1);     // Normal mode
    WriteData(0x05);   
    WriteData(0x3C);   
    WriteData(0x3C);   
    
    WriteComm(0xB2);     // Idle mode
    WriteData(0x05);   
    WriteData(0x3C);   
    WriteData(0x3C);   
    
    WriteComm(0xB3);     // Partial mode
    WriteData(0x05);   
    WriteData(0x3C);   
    WriteData(0x3C);   
    WriteData(0x05);   
    WriteData(0x3C);   
    WriteData(0x3C);   
    
    WriteComm(0xB4);     // Dot inversion
    WriteData(0x03);  
     
    WriteComm(0xC0);     
    WriteData(0x28);   
    WriteData(0x08);   
    WriteData(0x84);   
    
    WriteComm(0xC1);     // VGH VGL
    WriteData(0xC5);   
    
    WriteComm(0xC2);     // Normal Mode
    WriteData(0x0D);   
    WriteData(0x00);   
    
    WriteComm(0xC3);     // Idle
    WriteData(0x8D);   
    WriteData(0x6A);   
    
    WriteComm(0xC4);     // Partial+Full
    WriteData(0x8D);   
    WriteData(0xEE);   
    
    WriteComm(0xC5);     // VCOM
    WriteData(0x1E);   
    
    WriteComm(0xE0);     
    WriteData(0x12);
    WriteData(0x1E);
    WriteData(0x02);
    WriteData(0x00);
    WriteData(0x06);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x04);
    WriteData(0x0E);
    WriteData(0x29);
    WriteData(0x38);
    WriteData(0x10);
    WriteData(0x13);
    WriteData(0x00);
    WriteData(0x00);
    
    WriteComm(0xE1);
    WriteData(0x0E);
    WriteData(0x1B);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x01);
    WriteData(0x0F);
    WriteData(0x2A);
    WriteData(0x3A);
    WriteData(0x10);
    WriteData(0x0B);
    WriteData(0x00);
    WriteData(0x00);
    
    WriteComm(0x3A);     
    WriteData(0x05);   
    
    WriteComm(0x36);     // BGR 
    WriteData(0xC8);   
    
    WriteComm(0x21);     // Display inversion
    
    WriteComm(0x29);     // Display on
    
   /*  WriteComm(0x2A);     // Set Column Address
    WriteData(0x00);   
    WriteData(0x1A);   // 26  
    WriteData(0x00);   
    WriteData(0x69);   // 105 
    
    WriteComm(0x2B);     // Set Page Address
    WriteData(0x00);   
    WriteData(0x01);   // 1
    WriteData(0x00);   
    WriteData(0xA0);   // 160 */
    
    WriteComm(0x2C);

    printf("Display window set to full 240x240\n");

    DelayMs(100);
    
    // 填充黑色
    LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0,LCD_H-1, BLACK);
    printf("LCD initialized successfully\n");
   // DelayMs(1000);
    // LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0,LCD_H-1, WHITE);
    // DelayMs(1000);
     //LCD_Display_FullScreen_2(epd_bitmap_);
     //LCD_Display_Image_DMA(0,0, 240, 57, epd_bitmap_);

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

// 显示images.h图片
/* static void LCD_Display_FullScreen_2(const uint16_t *flash_address)
{
    uint32_t total_pixels = 240 * 57;
    uint32_t i;

    LCD_Address_Set(0, 0, 239, 56);

    LCD_CS_Clr();
    for (i = 0; i < total_pixels; i++) {
        spi_master_write(&spi_master, flash_address[i] >> 8);
        spi_master_write(&spi_master, flash_address[i] & 0xFF);
    }
    LCD_CS_Set();
}  */

// 背光控制
void LCD_BacklightOnOff(uint8_t onoff)
{
    if (onoff) {
        LCD_BL_Set();
    } else {
        LCD_BL_Clr();
    }
}