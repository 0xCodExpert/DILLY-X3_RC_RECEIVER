#ifndef INC_DRV_FUTABA_H_
#define INC_DRV_FUTABA_H_

#include <stdbool.h>

#include "typedefs.h"
#include "drv_hotrc.h"
#include "prv_uart.h"

#define FUTABA_BUFFER_LENGTH   25
#define CHANNEL_LENGTH  18

#define MSG_HEADER  0x0F
#define MSG_FOOTER_1 0x00
#define MSG_FOOTER_2 0x04
#define MSG_11BITCUT  0x07FF

#define NOT_INIT_TIMEOUT 1000
#define FUTABA_ERROR_VALUE 1121

void FutabaStateMachine(ControllerInfo *controller_info);
bool IsFutabaConnected(void);

extern uint8_t futaba_raw_buffer[50];
extern RingBuffer FutabaRxBuffer;

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

#endif  // INC_DRV_FUTABA_H_

