#ifndef INC_SOFTWARE_UART_H_
#define INC_SOFTWARE_UART_H_

#include "main.h"

#define BAUD_RATE_115200_DELAY (1000000U/115200)

void DelayMicroSeconds(uint32_t micro_seconds);

extern TIM_HandleTypeDef htim2;
void DelayUs(uint32_t udelay);
void UartDelay();
void TransmitSoftUart(char c[]);
void SendUartBit(uint8_t b);

#endif /* INC_SOFTWARE_UART_H_ */
