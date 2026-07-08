#ifndef USART_H
#define USART_H

#include "head.h"

void uart_received(void);
void LQ_USART_Init(void);
int uart_send(const char *str);
int uart_send_bytes(const uint8_t *data, int len);

#endif // USART_H