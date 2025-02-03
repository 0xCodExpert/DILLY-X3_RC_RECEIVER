#ifndef INC_TYPEDEFS_H_
#define INC_TYPEDEFS_H_

#include <stdbool.h>
#include <stdint.h>

#define NUM_OF_CH 8
#define UART_BUFFER_SIZE 128

typedef struct {
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} RingBuffer;

typedef enum {
  kNoError,
  kPacketError,
  kNotInitialized,
  kUartError,
} ControllerErrorCodeEnum;

typedef enum {
  kNoController,
  kExtCtrl,
  kFutaba,
  kHotrc
} ControllerTypeEnum;

typedef struct {
  uint8_t count;
  bool is_connected;
  uint32_t watchdog_timer;
} InstallDetection;

typedef struct {
  ControllerErrorCodeEnum error_code;
  ControllerTypeEnum controller_type;
  uint16_t *packets;
} ControllerInfo;

#endif  // INC_TYPEDEFS_H_
