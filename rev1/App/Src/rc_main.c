#include "rc_main.h"

uint16_t packets[8] = { 0, };

ControllerInfo rc_control = { kNoError, kNoController, packets };

uint8_t pass_filter_count = 0;
bool is_zero_data_in = false;

void MainStateMachine(void) {
  SetFutabaStatusLed(false);
  SetHotrcStatusLed(false);

  if (IsExternalControllerConnected()) {
    SetFutabaStatusLed(true);
    SetHotrcStatusLed(true);
    ExtCtrlStateMachine(&rc_control);
  } else if (IsFutabaConnected()) {
    SetFutabaStatusLed(true);
    FutabaStateMachine(&rc_control);
  } else if (IsHotrcControllerConnected()) {
    SetHotrcStatusLed(true);
    HotrcStateMachine(&rc_control);
  } else {
    rc_control.controller_type = kNoController;
    rc_control.error_code = kNotInitialized;
    SetBlueDebugLed(false);
  }

  if (rc_control.error_code == kNoError) {
    SetRedDebugLed(false);
    ToggleDebugBlueLed();
    TransmitPackets(rc_control.packets);
  } else if (rc_control.error_code == kNotInitialized ||
             rc_control.error_code == kPacketError) {
    ToggleDebugRedLed();
    ToggleDebugBlueLed();
  } else if (rc_control.error_code == kUartError) {
    SetBlueDebugLed(false);
    ToggleDebugRedLed();

    CheckControllerReset(rc_control.controller_type);
  }
}

// Checks if input value is in valid range.
bool CheckRange(uint16_t input_value[], ControllerTypeEnum controller_type) {
  // HOTRC controller has only 6 channels,
  // but to keep the protocol, dummy data will be added.
  // Still, only 6 values are to be verified.

  uint8_t num_channels_to_validate = (controller_type == kHotrc) ? 6 : 8;
  for (int i = 0; i < num_channels_to_validate; ++i) {
    if (input_value[i] > STICK_VALUE_MAX || input_value[i] < STICK_VALUE_MIN) {
      return false;
    }
    if (controller_type == kHotrc && i >= 2) {
      // Channel 3 to 6 of HOTRC receiver generates value
      // "1500" even when the controller is off.
      // This state must be detected as "invalid".
      // Notice that this "1500" value cannot be generated
      // while the controller is on, meaning it is safe to filter this value.
      if (input_value[i] == CH_NOT_CONNECTED) {
        return false;
      }
    }
  }
  return true;
}

void TransmitPackets(uint16_t *packets) {
  // Transmit packets based on ("," , "#") head detection protocol.
  char packets_transmit[50];

  Rs232Sw(true);
  if (!is_zero_data_in &&
      packets[0] != 0 && packets[1] != 0 && packets[3] != 0 && packets[7] != 0) {
    sprintf(packets_transmit, "%d,%d,%d,%d,%d,%d,%d,%d#", packets[0], packets[1],
        packets[2], packets[3], packets[4], packets[5], packets[6], packets[7]);
    HAL_UART_Transmit(&UART_RS232, (const uint8_t*) packets_transmit, 40, 1);
  } else if (is_zero_data_in && pass_filter_count > 30) {
    is_zero_data_in = false;
    pass_filter_count = 0;
  } else {
    is_zero_data_in = true;
    pass_filter_count++;
  }
}

uint8_t GetChecksum(uint8_t *byte_arr, uint8_t length) {
  uint8_t checksum;
  checksum = byte_arr[0];

  for (int i = 1; i < length - 1; ++i) {
    checksum ^= byte_arr[i];
  }

  return checksum;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART_RS232.Instance) {
    // When transmit is complete, RS232 is switched off for safety.
    Rs232Sw(false);
  }
}

// This function related to TRS3221E(IC) automatic power-down control.
// For saving driver power.
// When FORCEON is low and FORCEOFF is high, enable.
// Auto power-dwon can be disable when FORCEON and FORCEOFF are high.
void Rs232Sw(bool is_switch_pressed) {
  if (is_switch_pressed) {
    HAL_GPIO_WritePin(FORCEOFF_N_GPIO_Port, FORCEOFF_N_Pin, 1);
    HAL_GPIO_WritePin(FORCEON_GPIO_Port, FORCEON_Pin, 1);
    HAL_GPIO_WritePin(EN_N_GPIO_Port, EN_N_Pin, 0);  // active low
  } else {
    HAL_GPIO_WritePin(FORCEOFF_N_GPIO_Port, FORCEOFF_N_Pin, 0);
    HAL_GPIO_WritePin(FORCEON_GPIO_Port, FORCEON_Pin, 0);
    HAL_GPIO_WritePin(EN_N_GPIO_Port, EN_N_Pin, 1);
  }
}

// ----- LED control -----
void SetFutabaStatusLed(bool is_switch_pressed) {
  HAL_GPIO_WritePin(STATUS_FUTABA_GPIO_Port, STATUS_FUTABA_Pin,
                    !is_switch_pressed);
}

void SetHotrcStatusLed(bool is_switch_pressed) {
  HAL_GPIO_WritePin(STATUS_HOTRC_GPIO_Port, STATUS_HOTRC_Pin,
                    !is_switch_pressed);
}

void SetRedDebugLed(bool is_switch_pressed) {
  HAL_GPIO_WritePin(DEBUG_LED_R_GPIO_Port, DEBUG_LED_R_Pin,
                    !is_switch_pressed);
}

void SetBlueDebugLed(bool is_switch_pressed) {
  HAL_GPIO_WritePin(DEBUG_LED_B_GPIO_Port, DEBUG_LED_B_Pin,
                    !is_switch_pressed);
}

void ToggleDebugRedLed(void) {
  static uint32_t timer = 0;

  if (HAL_GetTick() - timer > LED_TOGGLE_TIME) {
    HAL_GPIO_TogglePin(DEBUG_LED_R_GPIO_Port, DEBUG_LED_R_Pin);
    timer = HAL_GetTick();
  }
}

void ToggleDebugBlueLed(void) {
  static uint32_t timer = 0;

  if (HAL_GetTick() - timer > LED_TOGGLE_TIME) {
    HAL_GPIO_TogglePin(DEBUG_LED_B_GPIO_Port, DEBUG_LED_B_Pin);
    timer = HAL_GetTick();
  }
}
