#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "stm32f0xx.h"

// USART1
void init_UART(uint32_t speed);

uint8_t serial_available(void);
uint8_t serial_read(void);
void serial_send_bytes(uint8_t *s, int n);
void print_int16(int16_t p_int);
void StartUSART(void);

//void TX_DMA(void);

#endif
