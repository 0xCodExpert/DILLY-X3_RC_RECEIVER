#include "drv_futaba.h"

static uint8_t* GetBuffer(void) {
  static uint8_t buffer[FUTABA_BUFFER_LENGTH];
  uint8_t header_index = 0;
  bool is_header_found = false;

  memset(buffer, 0, (FUTABA_BUFFER_LENGTH * sizeof(uint8_t)));
  // Buffer has to be cleared so that
  // it will automatically remove the risk of UART not working.

  for (int i = 0; i < FUTABA_BUFFER_LENGTH * 2; ++i) {
    unsigned char data = ReadUart(&FutabaRxBuffer);
    if (data == MSG_HEADER) {
      // Look for header message in the ring buffer.
      is_header_found = true;
    }
    if (is_header_found) {
      // If header message is found, start saving buffer from the index.
      buffer[header_index] = data;
      header_index++;
    }
    if (header_index >= FUTABA_BUFFER_LENGTH) {
      // If buffer is filled, stop saving.
      break;
    }
  }

  return (uint8_t*) buffer;
}

static bool IsBufferValid(uint8_t buffer[]) {
  uint8_t header = buffer[0];
  uint8_t footer_1 = buffer[FUTABA_BUFFER_LENGTH - 1];
  uint8_t footer_2 = buffer[FUTABA_BUFFER_LENGTH - 1] & 0x0F;

  return (((header == MSG_HEADER) && (footer_1 == MSG_FOOTER_1))
      || (footer_2 == MSG_FOOTER_2));
}

static uint16_t* FutabaSignalConvert(uint8_t buffer[]) {
  static uint16_t channels[CHANNEL_LENGTH];

  channels[0] = (uint16_t) ((buffer[1] | buffer[2] << 8) & MSG_11BITCUT);
  channels[1] = (uint16_t) ((buffer[2] >> 3 | buffer[3] << 5) & MSG_11BITCUT);
  channels[2] = (uint16_t) ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10)
								& MSG_11BITCUT);
  channels[3] = (uint16_t) ((buffer[5] >> 1 | buffer[6] << 7) & MSG_11BITCUT);
  channels[4] = (uint16_t) ((buffer[6] >> 4 | buffer[7] << 4) & MSG_11BITCUT);
  channels[5] = (uint16_t) ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9)
								& MSG_11BITCUT);
  channels[6] = (uint16_t) ((buffer[9] >> 2 | buffer[10] << 6) & MSG_11BITCUT);
  channels[7] = (uint16_t) ((buffer[10] >> 5 | buffer[11] << 3) & MSG_11BITCUT);
  channels[8] = (uint16_t) ((buffer[12] | buffer[13] << 8) & MSG_11BITCUT);
  channels[9] = (uint16_t) ((buffer[13] >> 3 | buffer[14] << 5) & MSG_11BITCUT);
  channels[10] = (uint16_t) ((buffer[14] >> 6 | buffer[15] << 2 |
                  buffer[16] << 10) & MSG_11BITCUT);
  channels[11] =
      (uint16_t) ((buffer[16] >> 1 | buffer[17] << 7) & MSG_11BITCUT);
  channels[12] =
      (uint16_t) ((buffer[17] >> 4 | buffer[18] << 4) & MSG_11BITCUT);
  channels[13] = (uint16_t) ((buffer[18] >> 7 | buffer[19] << 1
      | buffer[20] << 9) & MSG_11BITCUT);
  channels[14] =
      (uint16_t) ((buffer[20] >> 2 | buffer[21] << 6) & MSG_11BITCUT);
  channels[15] =
      (uint16_t) ((buffer[21] >> 5 | buffer[22] << 3) & MSG_11BITCUT);
  channels[16] = buffer[23] & (1 << 0);
  channels[17] = buffer[23] & (1 << 1);

  return (uint16_t*) channels;
}

static uint16_t* ScaleStickInput(uint16_t raw_value[]) {
  // Stick input must be scaled to PWM duty cycle converted in micro seconds.
  // For example, when lever is at neutral, the value should be 1522 (1522us).
  const int kShiftValue = 880;
  const float kGainValue = 0.625;
  static uint16_t scaled_values[NUM_OF_CH];

  memset(scaled_values, 0, NUM_OF_CH * sizeof(uint16_t));

  for (int i = 0; i < NUM_OF_CH; ++i) {
    scaled_values[i] = (uint16_t) (kGainValue * raw_value[i] + kShiftValue);
  }

  return (uint16_t*) scaled_values;
}

void FutabaStateMachine(ControllerInfo *controller_info) {
  static uint32_t timer = 0;
  controller_info->error_code = kNoError;
  controller_info->controller_type = kFutaba;
  memset(controller_info->packets, 0, (NUM_OF_CH * sizeof(uint16_t)));

  uint8_t *buffer;
  uint16_t *channels;
  uint16_t *scaled_channels;

  if (IsUartDataAvailable(&FutabaRxBuffer) > FUTABA_BUFFER_LENGTH * 2) {
    // To stably receive Futaba packets, wait for two chunks of buffer.
    buffer = GetBuffer();
    bool packet_error_status =
        ((buffer[23] & (1 << 2)) || (buffer[23] & (1 << 3)));

    if (IsBufferValid(buffer) && !packet_error_status) {
      timer = HAL_GetTick();
      // If buffer is okay and no packet error is detected,
      // Convert buffer to normal packets.
      channels = FutabaSignalConvert(buffer);
      scaled_channels = ScaleStickInput(channels);

      // These lines are for debugging with old model.
      // Delete it when you use r7008sb.
      scaled_channels[4] += 200;
      scaled_channels[6] += 200;

      if (!CheckRange(scaled_channels, controller_info->controller_type)) {
        // Check the converted packets again.
        // If packets are NOT within normal range, error must be followed.
        controller_info->error_code = kPacketError;
      }

    } else {
      // If buffer is NOT okay or packet error is detected,
      // error status must be followed.
      controller_info->error_code = kPacketError;
    }
    if (controller_info->error_code == kNoError) {
      // If there is no error in the packets, the packets shall be transferred
      // to main state machine to be transmitted.
      controller_info->packets = scaled_channels;
    }
  } else {
    // If UART buffer is not filled enough,
    // error must be followed.
    controller_info->error_code = kNotInitialized;

    // If "Not Initialized" state is kept for 1000 milliseconds,
    // it must be UART hardware error.
    if (HAL_GetTick() - timer > NOT_INIT_TIMEOUT) {
      controller_info->error_code = kUartError;
      timer = HAL_GetTick();
    }
  }
}

bool IsFutabaConnected(void) {
  static uint8_t count = 0;
  static uint32_t timer = 0;
  static bool is_connected = false;

  HAL_UART_StateTypeDef uart_state = HAL_UART_GetState(&UART_FUTABA);
  bool is_uart_full = (bool) ((UART_FUTABA.Instance->SR & 0b100000) >> 5);

  if (UART_FUTABA.ErrorCode != 0 && is_uart_full) {
    // If UART register shows error code while UART buffer is full,
    // the receiver must be installed again.
    // To re-begin the UART reception, reset UART peripheral.
    ResetFutabaUart();
  } else {
    if (uart_state == HAL_UART_STATE_BUSY_RX
        && ((UART_FUTABA.Instance->SR & 0b10000) >> 4)) {
      // If UART state is busy and RXNE is 1,
      // judge as the receiver is successfully installed.
      timer = HAL_GetTick();
      if (count >= 10) {
        // Count up to 10 for stabilization.
        is_connected = true;
      } else if (!is_connected) {
        count++;
      }
    }
    // If there is no signal coming in for 1000 milliseconds,
    // judge as Futaba controller is not installed.
    if (HAL_GetTick() - timer > NOT_INIT_TIMEOUT) {
      is_connected = false;
      count = 0;
    }
  }

  return is_connected;
}

