#ifndef _WS2812_H_
#define _WS2812_H_

#include "stm32f0xx.h"

void ws2812_init(uint8_t len);
void ws2812_send();
void ws2812_set_pixel(uint16_t pixel, uint8_t R, uint8_t G, uint8_t B);

#endif
