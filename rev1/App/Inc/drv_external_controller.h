#ifndef INC_DRV_EXTERNAL_CONTROLLER_H_
#define INC_DRV_EXTERNAL_CONTROLLER_H_

#include "typedefs.h"
#include "prv_uart.h"
#include "rc_main.h"

#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>

#define UART_BUFFER_LENGTH 40
#define NOT_INIT_TIMEOUT 1000

#define ZERO_COMPARE_BUFFER_SIZE 4
#define COMMA_COMPARE_BUFFER_SIZE 7

void ExtCtrlStateMachine(ControllerInfo *controller_info);
bool IsExternalControllerConnected(void);

extern RingBuffer ExtCtrlRxBuffer;


#endif  // INC_DRV_EXTERNAL_CONTROLLER_H_
