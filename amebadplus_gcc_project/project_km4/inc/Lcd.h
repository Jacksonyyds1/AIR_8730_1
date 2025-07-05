
#ifndef __LCD_RTL8721DCM_H__
#define __LCD_RTL8721DCM_H__

#include "ameba_soc.h"
#include "os_wrapper.h"

// LCD尺寸定义
 #define LCD_W               240
#define LCD_H                240

//ST7789V2 偏移量
#define ST7789V2_X_OFFSET    0
#define ST7789V2_Y_OFFSET    80

// 画笔颜色定义
#define WHITE               0xFFFF
#define BLACK               0x0000	  
#define BLUE                0x001F  
#define BRED                0XF81F
#define GRED                0XFFE0
#define GBLUE               0X07FF
#define RED                 0xF800  // RGB:FF1100
#define ORANGE              0xFC00  // RGB:FF8000
#define PINK                0XFAB9  // RGB:FF57CA
#define PURPLE              0X929F  // RGB:9752FF
#define MAGENTA             0xF81F
#define GREEN               0x0721  // RGB:00E50B
#define CYAN                0x7FFF
#define YELLOW              0xFEE0  // RGB:FFDD00
#define BROWN               0XBC40  // 棕色
#define BRRED               0XFC07  // 棕红色
#define GRAY                0X8430  // 灰色
#define DARKBLUE            0X01CF  // 深蓝色
#define LIGHTBLUE           0X7D7C  // 浅蓝色  
#define GRAYBLUE            0X5458  // 灰蓝色
#define LIGHTGREEN          0X841F  // 浅绿色
#define LGRAY               0XC618  // 浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE           0XA651  // 浅灰蓝色(中间层颜色)
#define LBBLUE              0X2B12  // 浅棕蓝色(选择条目的反色)

// 坐标结构体
typedef struct {
    uint16_t x_axis_value;
    uint16_t y_axis_value;
} STR_COORDINATE;

// 函数声明
void WriteData(uint8_t dat);
void WriteComm(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t dat);
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Fill_FixedColor(uint8_t xsta, uint8_t xend, uint8_t ysta, uint8_t yend, uint16_t color);
void LCD_Fill_FixedColor_Simple(uint8_t xsta, uint8_t xend, uint8_t ysta, uint8_t yend, uint16_t color);
void LCD_Display_FullScreen(uint16_t *flash_address);
void LCD_BacklightOnOff(uint8_t onoff);
void DisplayLCD_Init(void);

#endif // __LCD_RTL8721DCM_H__