#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/*��������*/
#define OLED_W_SCL(x)        GPIO_WriteBit(GPIOD, GPIO_Pin_13, (BitAction)(x))
#define OLED_W_SDA(x)        GPIO_WriteBit(GPIOD, GPIO_Pin_12, (BitAction)(x))

/*���ų�ʼ��*/
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
  * @brief  I2C��ʼ
  * @param  ��
  * @retval ��
  */
void OLED_I2C_Start(void) {
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/**
  * @brief  I2Cֹͣ
  * @param  ��
  * @retval ��
  */
void OLED_I2C_Stop(void) {
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/**
  * @brief  I2C����һ���ֽ�
  * @param  Byte Ҫ���͵�һ���ֽ�
  * @retval ��
  */
void OLED_I2C_SendByte(uint8_t Byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        OLED_W_SDA(Byte & (0x80 >> i));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);    //�����һ��ʱ�ӣ�������Ӧ���ź�
    OLED_W_SCL(0);
}

/**
  * @brief  OLEDд����
  * @param  Command Ҫд�������
  * @retval ��
  */
void OLED_WriteCommand(uint8_t Command) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);        //�ӻ���ַ
    OLED_I2C_SendByte(0x00);        //д����
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/**
  * @brief  OLEDд����
  * @param  Data Ҫд�������
  * @retval ��
  */
void OLED_WriteData(uint8_t Data) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);        //�ӻ���ַ
    OLED_I2C_SendByte(0x40);        //д����
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED���ù��λ��
  * @param  y �����Ͻ�Ϊԭ�㣬���·�������꣬��Χ��0~7
  * @param  x �����Ͻ�Ϊԭ�㣬���ҷ�������꣬��Χ��0~127
  * @retval ��
  */
void OLED_Set_Pos(unsigned char x, unsigned char y) {
    OLED_WriteCommand(0xB0 | y);                    //����yλ��
    OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4));    //����xλ�ø�4λ
    OLED_WriteCommand(0x00 | (x & 0x0F));            //����Xλ�õ�4λ
}

/**
  * @brief  OLED����
  * @param  ��
  * @retval ��
  */
void OLED_Clear(void) {
    u8 i, n;
    //������ʾ
    for (i = 0; i < 8; i++) {
        OLED_WriteCommand(0xb0 + i);    //����ҳ��ַ��0~7��
        OLED_WriteCommand(0x00);      //������ʾλ�á��е͵�ַ
        OLED_WriteCommand(0x10);      //������ʾλ�á��иߵ�ַ
        for (n = 0; n < 128; n++)
            OLED_WriteData(0);
    }
}

/**
 * ��ָ��λ����ʾһ���ַ�,���������ַ�
 * @param x  0~127
 * @param y  0~63
 * @param chr
 * @param Char_Size  ѡ������ 16/12
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size) {
    unsigned char c = 0, i = 0;
    c = chr - ' ';//�õ�ƫ�ƺ��ֵ
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
 * ��ָ��λ����ʾһ���ַ�,���������ַ�
 * @param x  0~127
 * @param y  0~63
 * @param chr
 * @param Char_Size  ѡ������ 16/12
 */
void OLED_ShowChar_12(uint8_t x, uint8_t y, uint8_t chr) {
    OLED_ShowChar(x, y, chr, 12);
}


/**
  * @brief  OLED��ʾ�ַ���
 * @param  x ��ʼ��λ�ã���Χ��0~127
  * @param  y ��ʼ��λ�ã���Χ��0~63
  * @param  String Ҫ��ʾ���ַ�������Χ��ASCII�ɼ��ַ�
  * @retval ��
  */
void OLED_ShowString(uint8_t x, uint8_t y, char *String) {
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar_12(x + i * 6 + 1, y, String[i]);
    }
}

/**
  * @brief  OLED�η�����
  * @retval ����ֵ����X��Y�η�
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y) {
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  OLED��ʾ���֣�ʮ���ƣ�������
  * @param  x ��ʼ��λ�ã���Χ��0~127
  * @param  y ��ʼ��λ�ã���Χ��0~63
  * @param  Number Ҫ��ʾ�����֣���Χ��0~4294967295
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~10
  * @retval ��
  */
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length) {
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar_12(x + i * 6, y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED��ʾ���֣�ʮ���ƣ�����������
 * @param  x ��ʼ��λ�ã���Χ��0~127
  * @param  y ��ʼ��λ�ã���Χ��0~63
  * @param  Number Ҫ��ʾ�����֣���Χ��-2147483648~2147483647
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~10
  * @retval ��
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
  * @brief  OLED��ʾ���֣�ʮ�����ƣ�������
  * @param  x ��ʼ��λ�ã���Χ��0~127
  * @param  y ��ʼ��λ�ã���Χ��0~63
  * @param  Number Ҫ��ʾ�����֣���Χ��0~0xFFFFFFFF
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~8
  * @retval ��
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
  * @brief  OLED��ʾ���֣������ƣ�������
  * @param  x ��ʼ��λ�ã���Χ��0~127
  * @param  y ��ʼ��λ�ã���Χ��0~63
  * @param  Number Ҫ��ʾ�����֣���Χ��0~1111 1111 1111 1111
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~16
  * @retval ��
  */
void OLED_ShowBinNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length) {
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar_12(x + i * 6 + 1, y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED��ʼ��
  * @param  ��
  * @retval ��
  */
void OLED_Init(void) {

    uint32_t i, j;
    //�ϵ���ʱ
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++);
    }

    OLED_I2C_Init();            //�˿ڳ�ʼ��

    OLED_WriteCommand(0xAE);//--display off                    �ر���ʾ��

    OLED_WriteCommand(0x00);//---set low column address        ���õ��е�ַ
    OLED_WriteCommand(0x10);//---set high column address       ���ø��е�ַ

    OLED_WriteCommand(0x40);//--set start line address         ������ʼ�е�ַ
    OLED_WriteCommand(0xB0);//--set page address               ����ҳ��ַ

    OLED_WriteCommand(0x81); // contract control               ���öԱȶȿ���
    OLED_WriteCommand(0xFF);//--128                            �Աȶ�ֵ����Ϊ128

    OLED_WriteCommand(0xA1);//set segment remap                ���ö���ӳ��
    OLED_WriteCommand(0xA6);//--normal / reverse               ����������ʾ/������ʾ
    OLED_WriteCommand(0xA8);//--set multiplex ratio(1 to 64)   ���ö�·���ñ��ʣ�1��64��
    OLED_WriteCommand(0x3F);//--1/32 duty                      ����DutyֵΪ1/32
    OLED_WriteCommand(0xC8);//Com scan direction               ����COMɨ�跽��

    OLED_WriteCommand(0xD3);//-set display offset              ������ʾƫ��
    OLED_WriteCommand(0x00);//                                 ������ʾ��ʼ��

    OLED_WriteCommand(0xD5);//set osc division                 ����������Ƶ
    OLED_WriteCommand(0x80);//                                 ����������ƵֵΪ80

    OLED_WriteCommand(0xD8);//set area color mode off          ����������ɫģʽ�ر�
    OLED_WriteCommand(0x05);//                                 ����������ɫģʽֵΪ05

    OLED_WriteCommand(0xD9);//Set Pre-Charge Period            ����Ԥ�������
    OLED_WriteCommand(0xF1);//                                 ����Ԥ�������ֵΪF1

    OLED_WriteCommand(0xDA);//set com pin configuartion        ����COM��������
    OLED_WriteCommand(0x12);//                                 ����COM��������ֵΪ12

    OLED_WriteCommand(0xDB);//set Vcomh                        ����Vcomh
    OLED_WriteCommand(0x30);//                                 ����VcomhֵΪ30

    OLED_WriteCommand(0x8D);//set charge pump enable           ���ó���ʹ��
    OLED_WriteCommand(0x14);//                                 ���ó���ʹ��ֵΪ14

    OLED_WriteCommand(0xAF);//--turn on oled panel             ����ʾ��

    OLED_Clear();                //OLED����
}

//����OLED��ʾ
void OLED_Display_On(void) {
    OLED_WriteCommand(0X8D);  //SET DCDC����
    OLED_WriteCommand(0X14);  //DCDC ON
    OLED_WriteCommand(0XAF);  //DISPLAY ON
}

//�ر�OLED��ʾ
void OLED_Display_Off(void) {
    OLED_WriteCommand(0X8D);  //SET DCDC����
    OLED_WriteCommand(0X10);  //DCDC OFF
    OLED_WriteCommand(0XAE);  //DISPLAY OFF
}