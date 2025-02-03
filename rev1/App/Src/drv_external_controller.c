#include "drv_external_controller.h"

char monitor_data = 0;
uint8_t check_found_header = 0;
uint8_t header_index = 0;

bool is_ext_data_valid = false;

// Received packet convert to int format.
void ConvertBufferToPacket(uint8_t* buffer, uint16_t* channels) {
  unsigned char str_temp[5];
  str_temp[4] = '\0';

  for (int i = 0; i < NUM_OF_CH; ++i) {
    memcpy(str_temp, &buffer[(i * 5) + 1], 4);
    channels[i] = atoi((const char *)str_temp);
  }
}

void GetBuffer(uint8_t* buffer) {
  check_found_header = 0;
  header_index = 0;

  // Buffer has to be cleared so that it will
  // automatically remove the risk of UART not working.
  memset(buffer, 0, (UART_BUFFER_LENGTH * sizeof(uint8_t)));

  for (int x = 0; x < UART_BUFFER_LENGTH * 2; ++x) {
    unsigned char data = ReadUart(&ExtCtrlRxBuffer);
    monitor_data = (char)data;
    // "#" is a footer. Call ReadUart one more time to shift to next data
    // (first value of the packet).

    if (monitor_data == '#') {
      check_found_header = 1;
    }

    if (check_found_header) {
      buffer[header_index] = data;
      header_index++;
    }

    if (header_index >= UART_BUFFER_LENGTH) {
      // If buffer is filled, stop saving.
      break;
    }
  }
}

// Checks incoming buffer is valid.
// Compatible with protocol with "," and "#" head char.
static bool IsBufferValid_ext(uint8_t* buffer) {
  uint8_t zero_compare_buf[ZERO_COMPARE_BUFFER_SIZE];
  uint8_t comma_compare_buf[COMMA_COMPARE_BUFFER_SIZE];

  for (int i = 0; i < COMMA_COMPARE_BUFFER_SIZE; i++) {
    comma_compare_buf[i] = buffer[UART_BUFFER_LENGTH - (5 * (i + 1))];
  }

  for (int i = 0; i < ZERO_COMPARE_BUFFER_SIZE; i++) {
    zero_compare_buf[i] = buffer[19 + (5 * i)];
  }

  for (int i = 0; i < COMMA_COMPARE_BUFFER_SIZE; i++) {
    if (comma_compare_buf[i] != ',') {
      return false;
    }
  }

  for (int i = 0; i < ZERO_COMPARE_BUFFER_SIZE; i++) {
    if (zero_compare_buf[i] != '0') {
      return false;
    }
  }

  return true;
}

bool IsExternalControllerConnected(void) {
  static uint8_t count = 0;
  static uint32_t timer = 0;
  static bool is_connected = false;
  volatile uint8_t is_valid = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);

  if (is_valid) {
    timer = HAL_GetTick();
    if (count >= 10) {
      is_connected = true;
    } else if (!is_connected) {
      count++;
    }
  } else if (HAL_GetTick() - timer > 1000) {
    is_connected = false;
    count = 0;
  }

  return is_connected;
}

void ExtCtrlStateMachine(ControllerInfo* controller_info) {
  static uint32_t timer = 0;
  static uint8_t buffer[UART_BUFFER_LENGTH];
  static uint16_t channels[NUM_OF_CH];

  controller_info->error_code = kNoError;
  controller_info->controller_type = kExtCtrl;

  if (IsUartDataAvailable(&ExtCtrlRxBuffer) > UART_BUFFER_LENGTH * 2) {
    GetBuffer(buffer);

    if (IsBufferValid_ext(buffer)) {
      timer = HAL_GetTick();
      ConvertBufferToPacket(buffer, channels);

      if (!CheckRange(channels, controller_info->controller_type)) {
        // If buffer is okay, check if signals are within valid range.
        controller_info->error_code = kPacketError;
        return;
      }
    } else {
      controller_info->error_code = kPacketError;
    }

    if (controller_info->error_code == kNoError) {
      // If there is no problem in buffer and signal range,
      // pass the values to main state machine.
      controller_info->packets = channels;
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
