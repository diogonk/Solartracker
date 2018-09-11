

#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include "tm4c1294ncpdt.h"

void Uart_Init(void);

void Uart_Transmit(uint32_t value);


#endif
