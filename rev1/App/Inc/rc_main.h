#ifndef INC_RC_MAIN_H_
#define INC_RC_MAIN_H_

#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal_tim.h"

#include "typedefs.h"
#include "prv_uart.h"
#include "drv_hotrc.h"
#include "drv_futaba.h"
#include "drv_external_controller.h"

#define STICK_VALUE_MAX 2200
#define STICK_VALUE_MIN 900
#define LED_TOGGLE_TIME 1000

void MainStateMachine(void) ;
bool CheckRange(uint16_t input_value[], ControllerTypeEnum controller_type);
uint8_t GetChecksum(uint8_t* byte_arr, uint8_t length);
void TransmitPackets(uint16_t *packets);
void Rs232Sw(bool is_switch_pressed);
void SetFutabaStatusLed(bool is_switch_pressed);
void SetHotrcStatusLed(bool is_switch_pressed);
void SetRedDebugLed(bool is_switch_pressed);
void SetBlueDebugLed(bool is_switch_pressed);
void ToggleDebugRedLed(void);
void ToggleDebugBlueLed(void);

#endif  // INC_RC_MAIN_H_

