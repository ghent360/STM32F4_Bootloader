#pragma once
#include <stdint.h>

uint8_t DWT_Delay_Init(void);
void DWT_Delay_DeInit(void);
void DWT_Delay_us(uint32_t microseconds);
void DWT_Delay_ms(uint32_t milliseconds);