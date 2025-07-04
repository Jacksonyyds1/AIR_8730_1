// RTL8721DCM LCD移植代码 - lcd.c
#include "ameba_soc.h"
#include "os_wrapper.h"
#include "spi_api.h"
#include "gpio_api.h"
#include "ameba_gdma.h"
#include "Lcd.h"

// SPI对象
static spi_t spi_master;
static GDMA_InitTypeDef GDMA_InitStruct;

// LCD控制引脚宏定义 - 需要根据实际硬件连接修改
#define LCD_RES_PIN         _PA_12
#define LCD_DC_PIN          _PA_22  


// SPI引脚定义 - 需要根据实际硬件连接修改
#define SPI_MOSI_PIN        _PA_23
#define SPI_MISO_PIN        _PA_20
#define SPI_SCLK_PIN        _PA_30
#define SPI_CS_PIN          _PA_21

// LCD尺寸定义
#define LCD_W               80
#define LCD_H               160


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



// DMA回调函数
static u32 LCD_DMA_Callback(void *para)
{
    UNUSED(para);

    u32 IsrTypeMap = 0;
    
    // 清除中断标志
    IsrTypeMap = GDMA_ClearINT(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum);
    
    if (IsrTypeMap & TransferType) {
        DMA_Complete_Flag = 1;
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

// DMA初始化
static void LCD_DMA_Init(void)
{
    // 初始化 GDMA 结构体
    GDMA_StructInit(&GDMA_InitStruct);
    
    // 设置 DMA 基本参数
    GDMA_InitStruct.GDMA_Index = 0;
    GDMA_InitStruct.GDMA_DIR = TTFCMemToPeri;
    GDMA_InitStruct.GDMA_SrcMsize = MsizeOne;
    GDMA_InitStruct.GDMA_SrcDataWidth = TrWidthOneByte;
    GDMA_InitStruct.GDMA_DstMsize = MsizeOne;
    GDMA_InitStruct.GDMA_DstDataWidth = TrWidthOneByte;
    GDMA_InitStruct.GDMA_SrcInc = IncType;
    GDMA_InitStruct.GDMA_DstInc = NoChange;
    GDMA_InitStruct.GDMA_IsrType = (TransferType | ErrType);
    GDMA_InitStruct.GDMA_ReloadSrc = 0;
    GDMA_InitStruct.GDMA_ReloadDst = 0;
    
    // 设置握手接口 - 如果使用SPI DMA，需要正确的握手接口
    GDMA_InitStruct.GDMA_SrcHandshakeInterface = 0;  // 内存到外设，源不需要握手
    GDMA_InitStruct.GDMA_DstHandshakeInterface = GDMA_HANDSHAKE_INTERFACE_SPI0_TX;  // 根据实际使用的SPI调整
    
    // 申请DMA通道 - 使用正确的API签名
    GDMA_InitStruct.GDMA_ChNum = GDMA_ChnlAlloc(GDMA_InitStruct.GDMA_Index, 
                                               LCD_DMA_Callback, 
                                               0, 
                                               6);
    
    if (GDMA_InitStruct.GDMA_ChNum == 0xFF) {
        printf("DMA channel allocation failed\n");
        return;
    }
    
    printf("DMA channel %d allocated successfully\n", GDMA_InitStruct.GDMA_ChNum);
}

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
    LCD_WR_REG(0x2a);  // 列地址设置
    LCD_WR_DATA(x1 + 0x1A);
    LCD_WR_DATA(x2 + 0x1A);
    LCD_WR_REG(0x2b);  // 行地址设置
    LCD_WR_DATA(y1 + 1);
    LCD_WR_DATA(y2 + 1);
    LCD_WR_REG(0x2c);  // 储存器写
}

// 填充固定颜色
void LCD_Fill_FixedColor(uint8_t xsta, uint8_t xend, uint8_t ysta, uint8_t yend, uint16_t color)
{
    uint32_t total_pixels;
    uint16_t *fill_buffer;
    uint32_t i;
    
    if (xsta >= LCD_W || ysta >= LCD_H || xend > LCD_W || yend > LCD_H || xsta >= xend || ysta >= yend)
        return;
    
    LCD_Address_Set(xsta, ysta, xend, yend);
    
    total_pixels = (xend - xsta + 1) * (yend - ysta + 1);
    
    // 使用DMA传输
    LCD_CS_Clr();
    
    // 创建填充缓冲区
    fill_buffer = (uint16_t *)rtos_mem_malloc(total_pixels * 2);
    if (fill_buffer != NULL) {
        // 填充颜色数据
        for (i = 0; i < total_pixels; i++) {
            fill_buffer[i] = color;
        }
        
        // 配置DMA
        GDMA_InitStruct.GDMA_SrcAddr = (u32)fill_buffer;
        GDMA_InitStruct.GDMA_DstAddr = (u32)&spi_master;  // 需要根据实际SPI寄存器地址修改
        GDMA_InitStruct.GDMA_BlockSize = total_pixels * 2;
        GDMA_InitStruct.GDMA_SrcDataWidth = TrWidthTwoBytes;
        GDMA_InitStruct.GDMA_DstDataWidth = TrWidthTwoBytes;
        
        GDMA_Init(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, &GDMA_InitStruct);
        
        DMA_Complete_Flag = 0;
        GDMA_Cmd(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, ENABLE);
        
        // 等待DMA完成
        while (DMA_Complete_Flag == 0) {
            rtos_time_delay_ms(1);
        }
        
        GDMA_Cmd(GDMA_InitStruct.GDMA_Index, GDMA_InitStruct.GDMA_ChNum, DISABLE);
        rtos_mem_free(fill_buffer);
    }
    
    LCD_CS_Set();
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
    LCD_BL_Set();
    
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
    
    WriteComm(0x2A);     // Set Column Address
    WriteData(0x00);   
    WriteData(0x1A);   // 26  
    WriteData(0x00);   
    WriteData(0x69);   // 105 
    
    WriteComm(0x2B);     // Set Page Address
    WriteData(0x00);   
    WriteData(0x01);   // 1
    WriteData(0x00);   
    WriteData(0xA0);   // 160
    
    WriteComm(0x2C);
    
    DelayMs(100);
    
    // 填充黑色
    LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0, LCD_H-1, RED);
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