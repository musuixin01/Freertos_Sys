#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *str);
void OLED_ShowChineseName(uint8_t x, uint8_t y);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

uint8_t OLED_ScanAddress(void);   /* returns 7-bit addr (0x3C/0x3D), 0=none */
uint8_t OLED_GetAddress(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
