#ifndef INC_DRV_HOTRC_H_
#define INC_DRV_HOTRC_H_

#include "rc_main.h"

#define NUM_OF_RC_PWM_CH 6

#define CH_NOT_CONNECTED 1500
#define CH_PUSHED 2000
#define CH_NOT_PUSHED 1000
#define WAIT_TIME_PUSH 1000
#define TIMEOUT_HOTRC_CONNTECTION 300
#define BUTTON_NOISE_CONSTANT 10

#define PUSH_BUTTON_LOW_THRESHOLD 1050
#define PUSH_BUTTON_DOOR_LOW_THRESHOLD 1200
#define PUSH_BUTTON_HIGH_THRESHOLD 1900

#define HCU_RX_BUTTON_LOW 1100
#define HCU_RX_BUTTON_HIGH 1940

#define HOTRC_CH1_2_MID_VALUE 1520

typedef enum {
  kCh1,
  kCh2,
  kCh3,
  kCh4,
  kCh5,
  kCh6,
} ChannelEnum;

typedef enum {
  kFalling,
  kRising,
} EdgeTypeEnum;

typedef struct {
  ChannelEnum ch_num;
  uint32_t count_start;
  uint32_t count_end;
  uint32_t width;
  TIM_HandleTypeDef *htim;
  HAL_TIM_ActiveChannel active_channel;
  uint32_t timer_ch;
  GPIO_TypeDef *port;
  uint16_t pin;
  TIM_TypeDef* timer;
} RcChannel;

typedef struct {
  bool state;
  uint32_t pushed_time;
} FunctionState;

void HotrcInitializedHw(void);
void HotrcDisableHw(void);
bool IsHotrcControllerConnected(void);
void HotrcStateMachine(ControllerInfo *controller_info);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#endif  // INC_DRV_HOTRC_H_

