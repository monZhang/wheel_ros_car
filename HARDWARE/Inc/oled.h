#ifndef __OLED_H
#define __OLED_H

#include "sys.h"
#include "system.h"

void OLED_I2C_Init(void);

void OLED_Init(void);

void OLED_Clear(void);

void OLED_ShowChar_12(u8 x, u8 y, u8 chr);

void OLED_ShowString(uint8_t x, uint8_t y, char *String);

void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length);

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t Number, uint8_t Length);

void OLED_ShowHexNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length);

void OLED_ShowBinNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length);


#endif

