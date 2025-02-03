#ifndef INC_SBUS_RECEIVE_H_
#define INC_SBUS_RECEIVE_H_

#include "main.h"
#include <stdbool.h>
#define BUFFER_LENGTH   25
#define CHANNEL_LENGTH  18

#define MSG_HEADER  0x0F
#define MSG_11BITCUT  0x07FF
#define NUM_CHANNELS_TRANSMIT  8
#define HCU_RESET_IDX  15

void InitializeHw();
void ProcessRcData(void);
void TransmitToHcu(uint16_t channels[]);
bool CheckRange(uint16_t input_value);
void ResetHcu(uint16_t reset_control_value);
uint16_t ScaleStickInput(uint16_t raw_value);

extern uint8_t buffer[BUFFER_LENGTH * 2];

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern IWDG_HandleTypeDef hiwdg;

#endif /* INC_SBUS_RECEIVE_H_ */
