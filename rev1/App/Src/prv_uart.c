#include "prv_uart.h"

RingBuffer FutabaRxBuffer = { { 0 }, 0, 0 };
RingBuffer ExtCtrlRxBuffer = { { 0 }, 0, 0 };

uint8_t futaba_raw_buffer[50];
uint8_t ext_ctrl_raw_buffer[50];

static void SaveInRingBuffer(unsigned char c, RingBuffer *PointerRingBuffer) {
  int i = (unsigned int) (PointerRingBuffer->head + 1) % UART_BUFFER_SIZE;

  if (i != PointerRingBuffer->tail) {
    PointerRingBuffer->buffer[PointerRingBuffer->head] = c;
    PointerRingBuffer->head = i;
  }
}

void InitReceiveUart(void) {
  HAL_UART_Receive_DMA(&UART_FUTABA, futaba_raw_buffer, 1);
  HAL_UART_Receive_DMA(&UART_EXT_CONTROLLER, ext_ctrl_raw_buffer, 1);
}

void ResetFutabaUart(void) {
  HAL_UART_Init(&UART_FUTABA);
  HAL_UART_Receive_DMA(&UART_FUTABA, futaba_raw_buffer, 1);
}

void ResetExtControllerUart(void) {
  HAL_UART_Init(&UART_EXT_CONTROLLER);
  HAL_UART_Receive_DMA(&UART_EXT_CONTROLLER, ext_ctrl_raw_buffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    SaveInRingBuffer(futaba_raw_buffer[0], &FutabaRxBuffer);
  } else if (huart->Instance == USART1) {
    SaveInRingBuffer(ext_ctrl_raw_buffer[0], &ExtCtrlRxBuffer);
  }
}
// Returns the first byte saved in ring buffer.
unsigned char ReadUart(RingBuffer *PointerRingBuffer) {
  if (PointerRingBuffer->head == PointerRingBuffer->tail) {
    // If the head isn't ahead of the tail, we don't have any characters
    return -1;
  } else {
    unsigned char c = PointerRingBuffer->buffer[PointerRingBuffer->tail];
    PointerRingBuffer->tail = (unsigned int) (PointerRingBuffer->tail + 1)
        % UART_BUFFER_SIZE;
    return c;
  }
}

// Returns the number of bytes saved in ring buffer.
int IsUartDataAvailable(RingBuffer *PointerRingBuffer) {
  return (uint16_t) (UART_BUFFER_SIZE + PointerRingBuffer->head
      - PointerRingBuffer->tail) % UART_BUFFER_SIZE;
}

void CheckControllerReset(ControllerTypeEnum controller_type){
  switch (controller_type) {
    case kExtCtrl:
      ResetExtControllerUart();
      break;
    case kFutaba:
      ResetFutabaUart();
      break;
    default:
      break;
  }
}

