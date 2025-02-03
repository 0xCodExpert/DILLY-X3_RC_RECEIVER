#ifndef INC_PRV_UART_H_
#define INC_PRV_UART_H_

#include <stm32f1xx_hal.h>
#include <stdio.h>
#include <string.h>

#include "typedefs.h"
#include "main.h"

#define UART_FUTABA (huart2)
#define UART_EXT_CONTROLLER (huart1)
#define UART_RS232 (huart1)

void InitReceiveUart(void);
int IsUartDataAvailable(RingBuffer *PointerRingBuffer);
unsigned char ReadUart(RingBuffer *PointerRingBuffer);
void ResetFutabaUart(void);
void ResetExtControllerUart(void);
void CheckControllerReset(ControllerTypeEnum controller_type);
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;


#endif  // INC_PRV_UART_H_
