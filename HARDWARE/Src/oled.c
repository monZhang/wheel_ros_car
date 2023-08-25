#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/*引脚配置*/
#define OLED_W_SCL(x)        GPIO_WriteBit(GPIOD, GPIO_Pin_13, (BitAction)(x))
#define OLED_W_SDA(x)        GPIO_WriteBit(GPIOD, GPIO_Pin_12, (BitAction)(x))

/*引脚初始化*/
void OLED_I2C_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    OLED_W_SCL(0);
    OLED_W_SDA(0);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void) {
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void) {
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        OLED_W_SDA(Byte & (0x80 >> i));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);    //额外的一个时钟，不处理应答信号
    OLED_W_SCL(0);
}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);        //从机地址
    OLED_I2C_SendByte(0x00);        //写命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);        //从机地址
    OLED_I2C_SendByte(0x40);        //写数据
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  x 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_Set_Pos(unsigned char x, unsigned char y) {
    OLED_WriteCommand(0xB0 | y);                    //设置y位置
    OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4));    //设置x位置高4位
    OLED_WriteCommand(0x00 | (x & 0x0F));            //设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void) {
    u8 i, n;
    //更新显示
    for (i = 0; i < 8; i++) {
        OLED_WriteCommand(0xb0 + i);    //设置页地址（0~7）
        OLED_WriteCommand(0x00);      //设置显示位置—列低地址
        OLED_WriteCommand(0x10);      //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WriteData(0);
    }
}

/**
 * 在指定位置显示一个字符,包括部分字符
 * @param x  0~127
 * @param y  0~63
 * @param chr
 * @param Char_Size  选择字体 16/12
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size) {
    unsigned char c = 0, i = 0;
    c = chr - ' ';//得到偏移后的值
    if (x > 127 - 1) {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16) {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WriteData(F8X16[c * 16 + i]);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WriteData(F8X16[c * 16 + i + 8]);
    } else {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WriteData(F6x8[c][i]);
    }
}

/**
 * 在指定位置显示一个字符,包括部分字符
 * @param x  0~127
 * @param y  0~63
 * @param chr
 * @param Char_Size  选择字体 16/12
 */
void OLED_ShowChar_12(uint8_t x, uint8_t y, uint8_t chr) {
    OLED_ShowChar(x, y, chr, 12);
}


/**
  * @brief  OLED显示字符串
 * @param  x 起始行位置，范围：0~127
  * @param  y 起始列位置，范围：0~63
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t x, uint8_t y, char *String) {
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar_12(x + i * 6 + 1, y, String[i]);
    }
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y) {
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  x 起始行位置，范围：0~127
  * @param  y 起始列位置，范围：0~63
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length) {
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar_12(x + i * 6, y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
 * @param  x 起始行位置，范围：0~127
  * @param  y 起始列位置，范围：0~63
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t Number, uint8_t Length) {
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0) {
        OLED_ShowChar_12(x, y, '+');
        Number1 = Number;
    } else {
        OLED_ShowChar_12(x, y, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++) {
        OLED_ShowChar_12(x + i * 6 + 1, y, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  x 起始行位置，范围：0~127
  * @param  y 起始列位置，范围：0~63
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length) {
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10) {
            OLED_ShowChar_12(x + i * 6 + 1, y, SingleNumber + '0');
        } else {
            OLED_ShowChar_12(x + i * 6 + 1, y, SingleNumber - 10 + 'A');
        }
    }
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  x 起始行位置，范围：0~127
  * @param  y 起始列位置，范围：0~63
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length) {
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar_12(x + i * 6 + 1, y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void) {

    uint32_t i, j;
    //上电延时
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++);
    }

    OLED_I2C_Init();            //端口初始化

    OLED_WriteCommand(0xAE);//--display off                    关闭显示屏

    OLED_WriteCommand(0x00);//---set low column address        设置低列地址
    OLED_WriteCommand(0x10);//---set high column address       设置高列地址

    OLED_WriteCommand(0x40);//--set start line address         设置起始行地址
    OLED_WriteCommand(0xB0);//--set page address               设置页地址

    OLED_WriteCommand(0x81); // contract control               设置对比度控制
    OLED_WriteCommand(0xFF);//--128                            对比度值设置为128

    OLED_WriteCommand(0xA1);//set segment remap                设置段重映射
    OLED_WriteCommand(0xA6);//--normal / reverse               设置正常显示/反向显示
    OLED_WriteCommand(0xA8);//--set multiplex ratio(1 to 64)   设置多路复用比率（1到64）
    OLED_WriteCommand(0x3F);//--1/32 duty                      设置Duty值为1/32
    OLED_WriteCommand(0xC8);//Com scan direction               设置COM扫描方向

    OLED_WriteCommand(0xD3);//-set display offset              设置显示偏移
    OLED_WriteCommand(0x00);//                                 设置显示起始行

    OLED_WriteCommand(0xD5);//set osc division                 设置振荡器分频
    OLED_WriteCommand(0x80);//                                 设置振荡器分频值为80

    OLED_WriteCommand(0xD8);//set area color mode off          设置区域颜色模式关闭
    OLED_WriteCommand(0x05);//                                 设置区域颜色模式值为05

    OLED_WriteCommand(0xD9);//Set Pre-Charge Period            设置预充电周期
    OLED_WriteCommand(0xF1);//                                 设置预充电周期值为F1

    OLED_WriteCommand(0xDA);//set com pin configuartion        设置COM引脚配置
    OLED_WriteCommand(0x12);//                                 设置COM引脚配置值为12

    OLED_WriteCommand(0xDB);//set Vcomh                        设置Vcomh
    OLED_WriteCommand(0x30);//                                 设置Vcomh值为30

    OLED_WriteCommand(0x8D);//set charge pump enable           设置充电泵使能
    OLED_WriteCommand(0x14);//                                 设置充电泵使能值为14

    OLED_WriteCommand(0xAF);//--turn on oled panel             打开显示屏

    OLED_Clear();                //OLED清屏
}

//开启OLED显示
void OLED_Display_On(void) {
    OLED_WriteCommand(0X8D);  //SET DCDC命令
    OLED_WriteCommand(0X14);  //DCDC ON
    OLED_WriteCommand(0XAF);  //DISPLAY ON
}

//关闭OLED显示
void OLED_Display_Off(void) {
    OLED_WriteCommand(0X8D);  //SET DCDC命令
    OLED_WriteCommand(0X10);  //DCDC OFF
    OLED_WriteCommand(0XAE);  //DISPLAY OFF
}