/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header; //CAN Bus Receive Header
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WAIT_HOST	0
#define IDLE		1
#define PAGE_PROG	2

typedef void (*pFunction)(void);

// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS              (uint32_t)0x08008000
#define MAIN_PROGRAM_PAGE_NUMBER                16
#define NUM_OF_PAGES                            (128 - MAIN_PROGRAM_PAGE_NUMBER)

// CAN identifiers
#define ACK_CAN_ID								 0x50D
#define DEVICE_CAN_ID                            0x10D
#define CMD_HOST_INIT                            0x01
#define CMD_PAGE_PROG                            0x02
#define CMD_BOOT                                0x03

#define CAN_RESP_OK                              1
#define CAN_RESP_ERROR                          2

uint8_t rx_data[8];
uint32_t can_mailbox;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */


static FLASH_EraseInitTypeDef eraseInitStruct;

pFunction JumpAddress;
bool is_jump_app = false;
uint8_t page_buffer[FLASH_PAGE_SIZE];
uint8_t page_index;
volatile int page_buffer_ptr;
volatile uint8_t blState;

int page_crc;
uint32_t crc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void JumpToApplication(void) {
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  for (int i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
    __DSB();
    __ISB();
  }

  JumpAddress = *(__IO pFunction*)(MAIN_PROGRAM_START_ADDRESS + 4);
  SCB->VTOR = (uint32_t)MAIN_PROGRAM_START_ADDRESS;

  __set_MSP(*(__IO uint32_t*)MAIN_PROGRAM_START_ADDRESS);

  JumpAddress();
}

void TransmitResponsePacket(uint8_t response) {
  tx_header.DLC = 1;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.StdId = DEVICE_CAN_ID;
  tx_header.TransmitGlobalTime = DISABLE;

  uint8_t csend[] = {response};
  HAL_CAN_AddTxMessage(&hcan, &tx_header, csend, &can_mailbox);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  if (is_jump_app)
    JumpToApplication();

  if (HAL_GetTick() > 10000 && blState == WAIT_HOST)
    JumpToApplication();
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
  }


  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14|STB_CAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB14 STB_CAN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|STB_CAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    Error_Handler();
  }

  if (rx_header.StdId != DEVICE_CAN_ID) {
    return;
  }

  if (blState == PAGE_PROG) {
    memcpy(&page_buffer[page_buffer_ptr], rx_data, rx_header.DLC);
    page_buffer_ptr += rx_header.DLC;

    if (page_buffer_ptr == FLASH_PAGE_SIZE) {
      HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
      crc =
          HAL_CRC_Calculate(&hcrc, (uint32_t*)page_buffer, FLASH_PAGE_SIZE / 4);

      if (crc == page_crc && page_index <= NUM_OF_PAGES) {
        HAL_FLASH_Unlock();

        uint32_t PageError = 0;

        eraseInitStruct.TypeErase = TYPEERASE_PAGES;
        eraseInitStruct.PageAddress =
            MAIN_PROGRAM_START_ADDRESS + page_index * FLASH_PAGE_SIZE;
        eraseInitStruct.NbPages = 1;

        HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);

        for (int i = 0; i < FLASH_PAGE_SIZE; i += 4) {
          HAL_FLASH_Program(
              TYPEPROGRAM_WORD,
              MAIN_PROGRAM_START_ADDRESS + page_index * FLASH_PAGE_SIZE + i,
              *(uint32_t*)&page_buffer[i]);
        }

        HAL_FLASH_Lock();

        TransmitResponsePacket(CAN_RESP_OK);
      } else {
        TransmitResponsePacket(CAN_RESP_ERROR);
      }

      blState = IDLE;

      HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    return;
  }

  switch (rx_data[0]) {
    case CMD_HOST_INIT:
      blState = IDLE;
      TransmitResponsePacket(CAN_RESP_OK);
      break;
    case CMD_PAGE_PROG:
      if (blState == IDLE) {
        memset(page_buffer, 0, FLASH_PAGE_SIZE);
        memcpy(&page_crc, &rx_data[2], sizeof(int));
        page_index = rx_data[1];
        blState = PAGE_PROG;
        page_buffer_ptr = 0;
      } else {
        // Should never get here
      }
      break;
    case CMD_BOOT:
      blState = CMD_BOOT;
      TransmitResponsePacket(CAN_RESP_OK);

      is_jump_app = true;
      break;
    default:
      break;
  }
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
}

void CAN_ErrorCallback() {
  Error_Handler();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
