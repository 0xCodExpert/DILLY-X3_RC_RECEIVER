#include "sbus_receive.h"
#include "software_uart.h"
#include <string.h>
#include <stdio.h>

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  __HAL_DMA_DISABLE(huart->hdmarx);
  huart->hdmarx->Instance->CNDTR = BUFFER_LENGTH * 2;
  __HAL_DMA_ENABLE(huart->hdmarx);

  ProcessRcData();

}

void ProcessRcData(void)
{
  /*
   * This callback function works as main receiving function;
   * this receives SBUS signal,
   * checks if it's valid,
   * converts it to packets by channels,
   * and transmits them through UART.
   */

  bool packet_error_status;
  bool is_all_values_in_range = true;  //default is True for AND operation.

  bool stop_flag = false;

  static uint8_t previous_byte;
  uint8_t buffer_index = 0;

  for (int i = 0; i < BUFFER_LENGTH * 2; ++i) {

    /*
     * This for loop tries to find header message in long buffer
     * in case of incoming signal being shifted in timing.
     */

    // This conditional statement searches for header and footer to prevent
    // header-like packet from being treated as a value.
    if (buffer[i] == MSG_HEADER
        && ((previous_byte == 0x00) || (previous_byte & 0x0F) == 0x04)) {
      buffer_index = i;
      stop_flag = true;
    }

    previous_byte = buffer[i];

    if (stop_flag == true) {
      break;
    }
  }

  if (stop_flag == true) {

    /*
     * buffer is parsed to channel signal array.
     * 11 bits per one channel are densely packed in the SBUS signal.
     */

    uint16_t channels[CHANNEL_LENGTH];

    channels[0] = (uint16_t) ((buffer[buffer_index + 1]
        | buffer[buffer_index + 2] << 8) & MSG_11BITCUT);

    channels[1] = (uint16_t) ((buffer[buffer_index + 2] >> 3
        | buffer[buffer_index + 3] << 5) & MSG_11BITCUT);

    channels[2] = (uint16_t) ((buffer[buffer_index + 3] >> 6
        | buffer[buffer_index + 4] << 2 | buffer[buffer_index + 5] << 10)
        & MSG_11BITCUT);

    channels[3] = (uint16_t) ((buffer[buffer_index + 5] >> 1
        | buffer[buffer_index + 6] << 7) & MSG_11BITCUT);

    channels[4] = (uint16_t) ((buffer[buffer_index + 6] >> 4
        | buffer[buffer_index + 7] << 4) & MSG_11BITCUT);

    channels[5] = (uint16_t) ((buffer[buffer_index + 7] >> 7
        | buffer[buffer_index + 8] << 1 | buffer[buffer_index + 9] << 9)
        & MSG_11BITCUT);

    channels[6] = (uint16_t) ((buffer[buffer_index + 9] >> 2
        | buffer[buffer_index + 10] << 6) & MSG_11BITCUT);

    channels[7] = (uint16_t) ((buffer[buffer_index + 10] >> 5
        | buffer[buffer_index + 11] << 3) & MSG_11BITCUT);

    channels[8] = (uint16_t) ((buffer[buffer_index + 12]
        | buffer[buffer_index + 13] << 8) & MSG_11BITCUT);

    channels[9] = (uint16_t) ((buffer[buffer_index + 13] >> 3
        | buffer[buffer_index + 14] << 5) & MSG_11BITCUT);

    channels[10] = (uint16_t) ((buffer[buffer_index + 14] >> 6
        | buffer[buffer_index + 15] << 2 | buffer[buffer_index + 16] << 10)
        & MSG_11BITCUT);

    channels[11] = (uint16_t) ((buffer[buffer_index + 16] >> 1
        | buffer[buffer_index + 17] << 7) & MSG_11BITCUT);

    channels[12] = (uint16_t) ((buffer[buffer_index + 17] >> 4
        | buffer[buffer_index + 18] << 4) & MSG_11BITCUT);

    channels[13] = (uint16_t) ((buffer[buffer_index + 18] >> 7
        | buffer[buffer_index + 19] << 1 | buffer[buffer_index + 20] << 9)
        & MSG_11BITCUT);

    channels[14] = (uint16_t) ((buffer[buffer_index + 20] >> 2
        | buffer[buffer_index + 21] << 6) & MSG_11BITCUT);

    channels[15] = (uint16_t) ((buffer[buffer_index + 21] >> 5
        | buffer[buffer_index + 22] << 3) & MSG_11BITCUT);

    channels[16] = buffer[buffer_index + 23] & (1 << 0);
    channels[17] = buffer[buffer_index + 23] & (1 << 1);

    packet_error_status =
        ((buffer[buffer_index + 23] & (1 << 2))
            || (buffer[buffer_index + 23] & (1 << 3)));

    for (int i = 0; i < NUM_CHANNELS_TRANSMIT; ++i) {
      // This for loop checks if all input values are in valid range.
      channels[i] = ScaleStickInput(channels[i]);
      is_all_values_in_range = (is_all_values_in_range
          && (CheckRange(channels[i])));
    }

    if (is_all_values_in_range == true && packet_error_status == false) {
      // If packets are good, transmit to HCU through UART.
      HAL_IWDG_Refresh(&hiwdg);
      TransmitToHcu(channels);
    }
  }
}

void ResetHcu(uint16_t reset_control_value) {
  /*
   * if reset control value goes above threshold,
   * this function physically control reset pin of HCU
   * in case of HCU being stuck.
   */
  const uint16_t kResetThreshold = 1500;

  if (reset_control_value > kResetThreshold) {
    HAL_GPIO_WritePin(HCU_RESET_GPIO_Port, HCU_RESET_Pin, true);
  } else {
    HAL_GPIO_WritePin(HCU_RESET_GPIO_Port, HCU_RESET_Pin, false);
  }
}

void TransmitToHcu(uint16_t channels[]) {
  /*
   * This function typecasts integer values to characters
   * and transmit to HCU through UART.
   */
  char packets[10];

  for (int i = 0; i < NUM_CHANNELS_TRANSMIT; ++i) {
    if (i != NUM_CHANNELS_TRANSMIT - 1) {
      sprintf(packets, "%d,", channels[i]);
    } else {
      sprintf(packets, "%d#", channels[i]);
    }
    TransmitSoftUart(packets);
    HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
  }
}

uint16_t ScaleStickInput(uint16_t raw_value)
{
  /*
   * Stick input must be scaled to PWM duty cycle converted in micro seconds.
   * For example, when lever is at neutral, the value should be 1522 (1522us).
   */
  const int kShiftValue = 880;
  const float kGainValue = 0.625;

  return (uint16_t) (kGainValue * raw_value + kShiftValue);
}

bool CheckRange(uint16_t input_value) {
  // This function checks if input value is in valid range.
  const int kStickValueMin = 1000;
  const int kStickValueMax = 2000;

  return (input_value >= kStickValueMin)
      && (input_value <= kStickValueMax);
}

void InitializeHw() {
  /*
   * This function is to initialize RS232 converter and peripherals.
   * FORCEOFF and FORCEON must be high to prevent automatic power down.
   */

  HAL_GPIO_WritePin(FORCEOFF_N_GPIO_Port, FORCEOFF_N_Pin, 1);
  HAL_GPIO_WritePin(FORCEON_GPIO_Port, FORCEON_Pin, 1);
  HAL_GPIO_WritePin(EN_N_GPIO_Port, EN_N_Pin, 0);  // active low

  HAL_TIM_Base_Start(&htim2);  // timer for micro-second delay

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer, BUFFER_LENGTH * 2);

  HAL_IWDG_Refresh(&hiwdg);

}
