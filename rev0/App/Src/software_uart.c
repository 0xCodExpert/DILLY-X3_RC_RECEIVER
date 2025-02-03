#include "software_uart.h"
#include <string.h>

void DelayMicroSeconds(uint32_t micro_seconds) {
  const uint32_t kTimerFrequency = 48;
  // Timer 1's frequency is 48 Mhz.
  // For accuracy, smaller unit is used for raw counting.

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  uint32_t wait_time = micro_seconds * kTimerFrequency;
  while (__HAL_TIM_GET_COUNTER(&htim2) < wait_time) {
    // Wait until counter reaches the desired time.
  };
}

void UartDelay() {
  // This function waits for delay specifically set to 115200 baud rate.
  DelayMicroSeconds(BAUD_RATE_115200_DELAY);
}

void SendUartBit(uint8_t b) {
  // This function physically controls GPIO pin for bit-banging UART.
  HAL_GPIO_WritePin(SBUS_CONV_GPIO_Port, SBUS_CONV_Pin, b);
  UartDelay();
}

void TransmitSoftUart(char c[]) {
  /*
   * This function sends byte for byte to receiver.
   */

  for (int i = 0; i < strlen(c); ++i) {

    SendUartBit(0);  // Start bit (LOW)

    uint8_t byte_to_send = c[i];

    for (int j = 0; j < 8; j++) {
      SendUartBit(byte_to_send & 0x01);
      byte_to_send >>= 1;
    }
    SendUartBit(1);  // Stop bit (High)
  }
  DelayMicroSeconds(BAUD_RATE_115200_DELAY * 5);  // Delay to distinguish packets
}

