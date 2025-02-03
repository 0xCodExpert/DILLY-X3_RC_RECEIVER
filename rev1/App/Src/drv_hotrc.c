#include "drv_hotrc.h"

uint16_t rc_values[NUM_OF_RC_PWM_CH];
uint8_t size_exter = 0;
uint8_t is_range_valid = 0;
bool sensor_data_save = false;
uint32_t button_low_time = 0;
uint32_t button_high_time = 0;

InstallDetection hotrc_detection = {0, false, 0};
RcChannel ch1 = {kCh1, 0, 0, 0, &htim1, HAL_TIM_ACTIVE_CHANNEL_1, TIM_CHANNEL_1,
                 GPIOA, GPIO_PIN_8, TIM1};

RcChannel ch2 = {kCh2, 0, 0, 0, &htim2, HAL_TIM_ACTIVE_CHANNEL_2, TIM_CHANNEL_2,
                 GPIOA, GPIO_PIN_9, TIM2};

RcChannel ch3 = {kCh3, 0, 0, 0, &htim3, HAL_TIM_ACTIVE_CHANNEL_3, TIM_CHANNEL_3,
                 GPIOA, GPIO_PIN_10, TIM3};

RcChannel ch4 = {kCh4, 0, 0, 0, &htim3, HAL_TIM_ACTIVE_CHANNEL_4, TIM_CHANNEL_4,
                 GPIOA, GPIO_PIN_11, TIM3};

RcChannel ch5 = {kCh5, 0, 0, 0, &htim3, HAL_TIM_ACTIVE_CHANNEL_1, TIM_CHANNEL_1,
                 GPIOA, GPIO_PIN_0, TIM3};

RcChannel ch6 = {kCh6, 0, 0, 0, &htim3, HAL_TIM_ACTIVE_CHANNEL_2, TIM_CHANNEL_2,
                 GPIOA, GPIO_PIN_1, TIM3};

uint16_t channels[NUM_OF_CH]; // For debugging.
uint16_t past_save_value[NUM_OF_RC_PWM_CH];

void HotrcInitializedHw(void) {
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
}

void HotrcDisableHw(void) {
  HAL_TIM_IC_Stop_IT(ch1.htim, ch1.timer_ch);
  HAL_TIM_IC_Stop_IT(ch2.htim, ch2.timer_ch);
  HAL_TIM_IC_Stop_IT(ch3.htim, ch3.timer_ch);
  HAL_TIM_IC_Stop_IT(ch4.htim, ch4.timer_ch);
  HAL_TIM_IC_Stop_IT(ch5.htim, ch5.timer_ch);
  HAL_TIM_IC_Stop_IT(ch6.htim, ch6.timer_ch);
}

// Converts push-button signals into latch signals
// for easier robot control.
static int IsDoorOpen(uint16_t input_value) {
  static uint32_t pushed_time = 0;
  static bool state = false;
  static bool rising_edge = false;
  static bool is_timer_excess = false;
  static bool falling_edge = false;
  static bool time_pending = false;

  if (input_value == HCU_RX_BUTTON_HIGH) {
    if (!rising_edge) {
      time_pending = true;
      rising_edge = true;
      pushed_time = HAL_GetTick();
      button_high_time = HAL_GetTick();
      falling_edge = false;
    }
  } else if (input_value == HCU_RX_BUTTON_LOW) {
    rising_edge = false;
    is_timer_excess = false;

    if (!falling_edge) {
      button_low_time = HAL_GetTick();
      falling_edge = true;
    }

    if (button_low_time - button_high_time >= WAIT_TIME_PUSH && time_pending) {
      state = !state;
      is_timer_excess = true;
    }
    if (50 < button_low_time - button_high_time &&
        button_low_time - button_high_time < 300 && time_pending) {
      sensor_data_save = !sensor_data_save;
    }

    time_pending = false;
  }

  return state;
}

// Case in HotRC, default calibration CH status is
// 1500 1500 1050 1050 1000 1100 (rc_vaule 0~5).
// You Must check this default signal value.

// Zero turn switch:   channels[2]        push "Button 3"
// Relay ON/OFF:       channels[3]        push "Button 4"
// Auto/manual switch: channels[4]        push "Button 5"
// Door switch:        channels[7]        hold "Button 6"
static uint16_t* HotrcSignalConvert(uint16_t* rc_values) {
  static const float kGain = -0.84;
  static const uint16_t kOriginOffset = 1500;
  static const uint16_t kRcOffset = 1520;
  memset(channels, 0, (NUM_OF_CH * sizeof(uint16_t)));

  channels[0] =
      (uint16_t)(((int)rc_values[0] - kOriginOffset) * -kGain + kRcOffset);
  channels[1] =
      (uint16_t)(((int)rc_values[1] - kOriginOffset) * kGain + kRcOffset);

  if (rc_values[2] < PUSH_BUTTON_LOW_THRESHOLD) {
    channels[2] = HCU_RX_BUTTON_LOW;
  } else if (rc_values[2] > PUSH_BUTTON_HIGH_THRESHOLD) {
    channels[2] = HCU_RX_BUTTON_HIGH;
  }

  if (rc_values[3] < PUSH_BUTTON_LOW_THRESHOLD) {
    channels[3] = HCU_RX_BUTTON_LOW;
  } else if (rc_values[3] > PUSH_BUTTON_HIGH_THRESHOLD) {
    channels[3] = HCU_RX_BUTTON_HIGH;
  }

  if (rc_values[4] < PUSH_BUTTON_LOW_THRESHOLD) {
    channels[4] = HCU_RX_BUTTON_LOW;

  } else if (rc_values[4] > PUSH_BUTTON_HIGH_THRESHOLD) {
    channels[4] = HCU_RX_BUTTON_HIGH;
  }

  channels[5] = 1520; // Reserved for Futaba

  if (sensor_data_save) {
    channels[6] = 1300;
  } else {
    channels[6] = 2140;
  }

  if (rc_values[5] < PUSH_BUTTON_DOOR_LOW_THRESHOLD) {
    channels[7] = HCU_RX_BUTTON_LOW;
  } else if (rc_values[5] > PUSH_BUTTON_HIGH_THRESHOLD) {
    channels[7] = HCU_RX_BUTTON_HIGH;
  }

  // Door switch

  channels[7] =
      IsDoorOpen(channels[7]) ? HCU_RX_BUTTON_LOW : HCU_RX_BUTTON_HIGH;
  }

  return (uint16_t*)channels;
}


void HotrcStateMachine(ControllerInfo* controller_info) {
  controller_info->error_code = kNoError;
  controller_info->controller_type = kHotrc;
  memset(controller_info->packets, 0, NUM_OF_CH);

  rc_values[0] = ch1.width;
  rc_values[1] = ch2.width;
  rc_values[2] = ch3.width;
  rc_values[3] = ch4.width;
  rc_values[4] = ch5.width;
  rc_values[5] = ch6.width;

  if (rc_values[0] > 1400 && rc_values[0] < 1600) {
    rc_values[0] = 1500;
  }

  if (rc_values[1] > 1400 && rc_values[1] < 1600) {
    rc_values[1] = 1500;
  }

  if (!CheckRange(rc_values, controller_info->controller_type)) {
    controller_info->error_code = kPacketError;
  }
  if (controller_info->error_code == kNoError) {
    if (size_exter == 0) {
      size_exter = 1;
    } else {
      size_exter = 0;
    }
    controller_info->packets = HotrcSignalConvert(rc_values);
  }
}

static void HotRcDetectionCounter(void) {
  hotrc_detection.watchdog_timer = HAL_GetTick();
  if (hotrc_detection.is_connected == false) {
    if (hotrc_detection.count < 10) {
      hotrc_detection.count++;
    } else {
      hotrc_detection.is_connected = true;
      button_high_time = HAL_GetTick();
      button_low_time = HAL_GetTick();
      hotrc_detection.count = 0;
    }
  }
}

// Returns whether HOTRC controller is installed or not.
// is_connected gets up in timer callback function,
// which is automatically called by peripherals.
// is_connected gets down
// when watchdog_timer is not reset by the timer callback function.
bool IsHotrcControllerConnected(void) {
  if (HAL_GetTick() - hotrc_detection.watchdog_timer >
      TIMEOUT_HOTRC_CONNTECTION) {
    hotrc_detection.is_connected = false;
    button_high_time = 0;
    button_low_time = 0;
  }

  return hotrc_detection.is_connected;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  if ((ch4.width < CH_NOT_PUSHED + BUTTON_NOISE_CONSTANT &&
       ch4.width > CH_NOT_PUSHED - BUTTON_NOISE_CONSTANT) ||
      (ch4.width < CH_PUSHED + BUTTON_NOISE_CONSTANT &&
       ch4.width > CH_PUSHED - BUTTON_NOISE_CONSTANT)) {
    HotRcDetectionCounter();
  }

  // TIM1
  // GPIOA PIN 8: Right <-> Left
  // TIM2
  // GPIOA PIN 9: Forward <-> backward
  if (GPIO_pin == GPIO_PIN_8) {
    ch1.width = __HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
  }

  if (GPIO_pin == GPIO_PIN_9) {
    ch2.width = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
  }

  // TIM3
  // GPIOA 10 : Button 3
  // GPIOA 11 : Button 4
  // GPIOA 0  : Button 5
  // GPIOA 1  : Button 6
  if (GPIO_pin == GPIO_PIN_10) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) {
      __HAL_TIM_SET_COUNTER(&htim3, 0);
    } else {
      ch3.width = __HAL_TIM_GET_COUNTER(&htim3);
    }
  }
  if (GPIO_pin == GPIO_PIN_11) {
    ch4.width = __HAL_TIM_GET_COUNTER(&htim3);
  }
  if (GPIO_pin == GPIO_PIN_0) {
    ch5.width = __HAL_TIM_GET_COUNTER(&htim3);
  }
  if (GPIO_pin == GPIO_PIN_1) {
    ch6.width = __HAL_TIM_GET_COUNTER(&htim3);
  }
}
