#include "zf_common_headfile.h"
#include "ti_msp_dl_config.h"
#include "OLED_Font.h"

/* ==================== 引脚配置 ==================== */
#define OLED_SCL_PIN        B13    // SCL 引脚
#define OLED_SDA_PIN        B12    // SDA 引脚

/* ==================== 延时函数 ==================== */
static void I2C_Delay_us(uint32_t us)
{
    volatile uint32_t count;
    while (us--)
    {
        for (count = 0; count < 10; count++);
    }
}

static void I2C_Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        I2C_Delay_us(1000);
    }
}

/* ==================== GPIO 操作宏 ==================== */
#define OLED_W_SCL(x)   do { gpio_set_level(OLED_SCL_PIN, x); I2C_Delay_us(1); } while(0)
#define OLED_W_SDA(x)   do { gpio_set_level(OLED_SDA_PIN, x); I2C_Delay_us(1); } while(0)

/* ==================== 引脚初始化 ==================== */
void OLED_I2C_Init(void)
{
    gpio_init(OLED_SCL_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(OLED_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    
    OLED_W_SCL(1);
    OLED_W_SDA(1);
    // I2C_Delay_us(10);  // 已关闭
}

/* ==================== I2C 时序函数 ==================== */

void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    I2C_Delay_us(2);
    OLED_W_SCL(1);
    I2C_Delay_us(2);
    OLED_W_SDA(0);
    I2C_Delay_us(2);
    OLED_W_SCL(0);
    I2C_Delay_us(2);
}

void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    I2C_Delay_us(2);
    OLED_W_SCL(1);
    I2C_Delay_us(2);
    OLED_W_SDA(1);
    I2C_Delay_us(2);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        I2C_Delay_us(1);
        OLED_W_SCL(1);
        I2C_Delay_us(2);
        OLED_W_SCL(0);
        I2C_Delay_us(1);
    }
    OLED_W_SCL(1);
    I2C_Delay_us(2);
    OLED_W_SCL(0);
    I2C_Delay_us(1);
}

void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x00);
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
    // I2C_Delay_us(10);  // 已关闭
}

void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
    // I2C_Delay_us(10);  // 已关闭
}

/* ==================== 显示控制函数 ==================== */

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* ==================== 数学辅助函数 ==================== */

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/* ==================== 数字显示函数 ==================== */

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ==================== OLED 初始化 ==================== */

void OLED_Init(void)
{
    I2C_Delay_ms(200);          // 上电延时（必须保留）
    
    OLED_I2C_Init();
    // I2C_Delay_ms(50);        // 已关闭
    
    OLED_WriteCommand(0xAE);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0x40);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0xA1);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0xC8);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    // I2C_Delay_ms(10);        // 已关闭
    
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xFF);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x30);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0xA4);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0xA6);
    // I2C_Delay_ms(10);        // 已关闭

    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    // I2C_Delay_ms(20);        // 已关闭

    OLED_WriteCommand(0xAF);
    // I2C_Delay_ms(20);        // 已关闭
        
    OLED_Clear();
    // I2C_Delay_ms(20);        // 已关闭
    
    //OLED_ShowString(1, 1, "OK");
}