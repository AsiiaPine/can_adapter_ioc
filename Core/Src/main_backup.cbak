/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <cstdint>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t channel;  // 1 for FDCAN1, 2 for FDCAN2
  bool isExtended;
  bool isRemote;
} can_message_t;

typedef struct {
  uint8_t command;
  uint8_t channel;
  uint8_t id[9];
  uint8_t dlc;
  uint8_t data[8];
} usb_can_frame_t;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t usb_rx_buffer[USB_BUFFER_SIZE];
uint8_t usb_tx_buffer[USB_BUFFER_SIZE];
uint8_t usb_rx_index = 0;
uint8_t usb_tx_index = 0;

static can_message_t can_rx_queue[CAN_MAX_MESSAGES];
static uint8_t can_rx_queue_head = 0;
static uint8_t can_rx_queue_tail = 0;

static uint32_t can1_rx_count = 0;
static uint32_t can2_rx_count = 0;
static uint32_t can1_tx_count = 0;
static uint32_t can2_tx_count = 0;

static uint8_t can1_terminator_enabled = 0;
static uint8_t can2_terminator_enabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void process_usb_command(uint8_t *data, uint16_t len);
static void send_can_message(can_message_t *msg);
static void receive_can_message(FDCAN_HandleTypeDef *hfdcan, uint8_t channel);
static void send_usb_response(uint8_t *data, uint16_t len);
static void update_led_status(void);
static void toggle_led(uint16_t pin);
static void send_can_to_usb(can_message_t *msg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printBits(size_t const size, void const * const ptr, char* const str)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    char* str_ptr = str;
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            snprintf(str_ptr, 2, "%d", byte);
            str_ptr++;
        }
    }
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  HAL_FDCAN_ConfigExtendedIdMask(&hfdcan1, 0x1FFFFFFFU);
  HAL_FDCAN_ConfigExtendedIdMask(&hfdcan2, 0x1FFFFFFFU);
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_GROUP_RX_FIFO1, FDCAN_INTERRUPT_LINE1);
  // Start CAN reception
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);

  /*
    TODO:
    1. Check HAL_FDCAN_ConfigRamWatchdog
    2. HAL_FDCAN_AddMessageToTxFifoQ
    3. HAL_FDCAN_ConfigRxFifoOverwrite
  */ 

  // Activate CAN RX notifications
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Initialize LED status
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Red LED off -- can error
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   // Green LED off -- can irq received
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);   // Blue LED off -- usb connected

  // Send initial status
  uint8_t init_msg[] = "USB-CAN Adapter Ready\r\n";
  CDC_Transmit_FS(init_msg, sizeof(init_msg) - 1);

  // Test: Send a test CAN message to trigger interrupt
  can_message_t test_msg;
  test_msg.isExtended = true;
  test_msg.isRemote = true;
  test_msg.id = 0x0c01ff;
  test_msg.dlc = 0;
  test_msg.channel = 1;
  send_can_message(&test_msg);

  // Debug: Check CAN state
  uint8_t can_state_msg[128];
  int state_len = snprintf((char*)can_state_msg, sizeof(can_state_msg),
      "CAN1 State: %lu, CAN2 State: %lu\r\n",
      hfdcan1.State, hfdcan2.State);
  CDC_Transmit_FS(can_state_msg, state_len);
  can_message_t test_messages[2];
  test_messages[0].id = 0x0c01ff;
  test_messages[0].dlc = 2;
  test_messages[0].channel = 1;
  test_messages[0].isExtended = true;
  test_messages[0].isRemote = true;
  test_messages[0].data[0] = 0x0060;
  test_messages[0].data[1] = 0x00;

  test_messages[1].id = 0x0c01ff;
  test_messages[1].dlc = 4;
  test_messages[1].channel = 1;
  test_messages[1].isExtended = true;
  test_messages[1].isRemote = false;
  test_messages[1].data[0] = 0x0020;
  test_messages[1].data[1] = 0x00;
  test_messages[1].data[2] = 0xB8;
  test_messages[1].data[3] = 0x0B;
  static int i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_time = HAL_GetTick();
  uint32_t last_send_time = HAL_GetTick();
  uint32_t last_loop_time = HAL_GetTick();
  while (1)
  {
    if (HAL_GetTick() - last_loop_time < 2) {
      HAL_IWDG_Refresh(&hiwdg);
      continue;
    }
    last_loop_time = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Feed watchdog
    if (HAL_GetTick() - last_time > 1000) {
      last_time = HAL_GetTick();
      CDC_Transmit_FS(init_msg, sizeof(init_msg) - 1);
    }

    // Update LED status
    update_led_status();

    // Test: Send periodic test CAN message for debugging
    if (HAL_GetTick() - last_send_time >= 1000) { // Every 1 second
      // can_message_t test_msg;
      // test_msg.id = 0x0c01ff;
      // test_msg.dlc = 4;
      // test_msg.isExtended = true;
      // test_msg.channel = 1;
      // // test_msg.channel ++;
      // // test_msg.channel %= 2;
      // test_msg.data[0] = 0xAA;
      // test_msg.data[1] = 0xBB;
      // test_msg.data[2] = 0xCC;
      // test_msg.data[3] = 0xDD;
      i = (i + 1) % 2; // Toggle between two test messages
      send_can_message(&test_messages[i]);

      // Send status via USB
      uint8_t status_msg[64];
      int len = snprintf((char*)status_msg, sizeof(status_msg),
          "CAN1: RX=%lu TX=%lu, CAN2: RX=%lu TX=%lu\r\n",
          can1_rx_count, can1_tx_count, can2_rx_count, can2_tx_count);
      CDC_Transmit_FS(status_msg, len);
      last_send_time = HAL_GetTick();
    }

    // Process any pending USB data
    if (usb_rx_index > 0) {
      process_usb_command(usb_rx_buffer, usb_rx_index);
      usb_rx_index = 0;
    }

    // Send any queued CAN messages
    while (can_rx_queue_head != can_rx_queue_tail) {
      char rx_msg[64];

      can_message_t *msg = &can_rx_queue[can_rx_queue_tail];

      send_can_to_usb(msg);
      can_rx_queue_tail = (can_rx_queue_tail + 1) % CAN_MAX_MESSAGES;
    }
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
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/* USER CODE BEGIN 4 */

/* USB-CAN Converter Helper Functions */

static void process_usb_command(uint8_t *data, uint16_t len)
{
    if (len < sizeof(usb_can_frame_t)) {
        return;
    }

    usb_can_frame_t *frame = (usb_can_frame_t*)data;

    switch (frame->command) {
        case CMD_CAN_SEND: {
            can_message_t msg;
            msg.id = strtoul((char*)frame->id, NULL, 16);
            msg.dlc = frame->dlc;
            msg.channel = frame->channel;
            memcpy(msg.data, frame->data, 8);
            send_can_message(&msg);
            break;
        }
        case CMD_CAN_STATUS: {
            uint8_t status_msg[64];
            int len = snprintf((char*)status_msg, sizeof(status_msg),
                "CAN1: RX=%lu TX=%lu, CAN2: RX=%lu TX=%lu\r\n",
                can1_rx_count, can1_tx_count, can2_rx_count, can2_tx_count);
            send_usb_response(status_msg, len);
            break;
        }
        case CMD_CAN_CONFIG: {
            if (frame->channel == 1) {
                can1_terminator_enabled = frame->data[0];
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 
                    can1_terminator_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
            } else if (frame->channel == 2) {
                can2_terminator_enabled = frame->data[0];
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 
                    can2_terminator_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            break;
        }
        case CMD_CAN_RESET: {
            can1_rx_count = 0;
            can1_tx_count = 0;
            can2_rx_count = 0;
            can2_tx_count = 0;
            uint8_t reset_msg[] = "Counters reset\r\n";
            send_usb_response(reset_msg, strlen((char*)reset_msg));
            break;
        }
    }
}

static void send_can_message(can_message_t *msg)
{
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = msg->id;
    tx_header.IdType = msg->isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = msg->isRemote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_header.DataLength = msg->dlc << 16;
    // 10  0000   0000  0000  0000
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    FDCAN_HandleTypeDef *hfdcan = (msg->channel == 1) ? &hfdcan1 : &hfdcan2;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, msg->data) == HAL_OK) {
        if (msg->channel == 1) {
            can1_tx_count++;
        } else {
            can2_tx_count++;
        }
    }
}

static void receive_can_message(FDCAN_HandleTypeDef *hfdcan, uint8_t channel)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        uint8_t rx_msg[64];
        int len = snprintf((char*)rx_msg, sizeof(rx_msg), "CAN%d: got a message\r\n", channel);
        CDC_Transmit_FS(rx_msg, len);
        // Check if we have space in the queue
        uint8_t next_head = (can_rx_queue_head + 1) % CAN_MAX_MESSAGES;
        if (next_head != can_rx_queue_tail) {
            can_message_t *msg = &can_rx_queue[can_rx_queue_head];
            msg->id = rx_header.Identifier;
            msg->dlc = rx_header.DataLength;
            msg->channel = channel;
            memcpy(msg->data, rx_data, 8);
            can_rx_queue_head = next_head;
            if (channel == 1) {
                can1_rx_count++;
            } else {
                can2_rx_count++;
            }
            return;
        }
    }
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
        uint8_t rx_msg[64];
        int len = snprintf((char*)rx_msg, sizeof(rx_msg), "CAN%d: got a message\r\n", channel);
        CDC_Transmit_FS(rx_msg, len);

        // Check if we have space in the queue
        uint8_t next_head = (can_rx_queue_head + 1) % CAN_MAX_MESSAGES;
        if (next_head != can_rx_queue_tail) {
            can_message_t *msg = &can_rx_queue[can_rx_queue_head];
            msg->id = rx_header.Identifier;
            msg->dlc = rx_header.DataLength;
            msg->channel = channel;
            memcpy(msg->data, rx_data, 8);
            can_rx_queue_head = next_head;
            can2_rx_count++;
        }
        return;
    }
    CDC_Transmit_FS("CAN: got here\r\n", 15);
}

static void send_can_to_usb(can_message_t *msg)
{
    usb_can_frame_t frame;
    frame.command = CMD_CAN_RECEIVE;
    frame.channel = msg->channel;

    // Extract each nibble (4 bits) from the ID
    for (int i = 0; i < 8; i++) {
      // Calculate shift amount (4 bits per nibble, starting from MSB)
      int shift = (7 - i) * 4;
      // Extract the nibble and store it
      frame.id[i] = (msg->id >> shift) & 0xF;
    }
    frame.dlc = msg->dlc;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = msg->data[i];
    }
    char init_id_str[9];

    sprintf((char*)init_id_str, "%X\n", msg->id);
    CDC_Transmit_FS(init_id_str, strlen((char*)init_id_str));

    // snprintf((char*)rx_msg, strlen((char*)rx_msg), "data: %lu\n", msg->data);
    // send_usb_response((uint8_t*)&frame, sizeof(frame));


    frame.dlc = msg->dlc;

    char rx_msg[64] = {};

    char dlc_str[8];
    snprintf(dlc_str, 8, "%X", (int)(msg->dlc));

    char cmd_str[2];
    snprintf(cmd_str, 2, "%X", frame.command);
    char data_str[64];
    for (int i = 0; i < 8; i++) {
      snprintf(data_str + i*2, 3, "%02X", frame.data[i]);
    }
    sprintf((char*)rx_msg, "cmd: %s id: %s dlc: %s data: %s\r\n", cmd_str, init_id_str, dlc_str, data_str);
    CDC_Transmit_FS((uint8_t*)rx_msg, strlen((char*)rx_msg));
    // send_usb_response((uint8_t*)&frame, sizeof(frame));
}

static void send_usb_response(uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS(data, len);
}

static void update_led_status(void)
{
}

static void toggle_led(uint16_t pin)
{
    HAL_GPIO_TogglePin(GPIOC, pin);
}

/* CAN Interrupt Callbacks */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  // Debug: Toggle green LED to indicate interrupt was called
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);

  uint8_t channel = 1;
  if (hfdcan == &hfdcan2) {
    channel = 2;
  }
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
      receive_can_message(hfdcan,channel);  // Channel 1 for FDCAN1
  }
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    receive_can_message(hfdcan, channel);  // Channel 2 for FDCAN2
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  // Debug: Toggle green LED to indicate interrupt was called
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
  uint8_t channel = 1;
  if (hfdcan == &hfdcan2) {
    channel = 2;
  }
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
      receive_can_message(hfdcan, channel + 2);  // Channel 1 for FDCAN1
  }
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    receive_can_message(hfdcan, channel+2);  // Channel 2 for FDCAN1
  }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    // Optional: Handle transmission events
    // UNUSED(hfdcan);
    uint8_t error_msg[19];

    if (hfdcan == &hfdcan1) {
      // Toggle red LED to indicate CAN1 error
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      // Send error message via USB
      memcpy(error_msg, "Retransmit: CAN1\r\n", sizeof(error_msg));
    } else if (hfdcan == &hfdcan2) {
      // Toggle red LED to indicate CAN2 error
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      // Send error message via USB
      memcpy(error_msg, "Retransmit: CAN2\r\n", sizeof(error_msg));
    }
    CDC_Transmit_FS(error_msg, sizeof(error_msg) - 1);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    // Handle CAN errors
    if (hfdcan == &hfdcan1) {
        // Toggle red LED to indicate CAN1 error
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        // Send error message via USB
        uint8_t error_msg[] = "CAN1 Error\r\n";
        CDC_Transmit_FS(error_msg, sizeof(error_msg) - 1);
    } else if (hfdcan == &hfdcan2) {
        // Toggle red LED to indicate CAN2 error
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        // Send error message via USB
        uint8_t error_msg[] = "CAN2 Error\r\n";
        CDC_Transmit_FS(error_msg, sizeof(error_msg) - 1);
    }
    uint8_t error_msg[19];
    snprintf((char*)error_msg, sizeof(error_msg), "CAN Error %d\r\n", (int)(hfdcan->ErrorCode));
    CDC_Transmit_FS(error_msg, sizeof(error_msg) - 1);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  // Fallback: Configure PLL manually
  __HAL_RCC_PLL_DISABLE();
  RCC->PLLCFGR = (RCC_PLLSOURCE_HSI << RCC_PLLCFGR_PLLSRC_Pos) |
                  (RCC_PLLM_DIV1 << RCC_PLLCFGR_PLLM_Pos) |
                  (9 << RCC_PLLCFGR_PLLN_Pos) |
                  (RCC_PLLP_DIV2 << RCC_PLLCFGR_PLLP_Pos) |
                  (RCC_PLLQ_DIV3 << RCC_PLLCFGR_PLLQ_Pos) |
                  (RCC_PLLR_DIV3 << RCC_PLLCFGR_PLLR_Pos);
  __HAL_RCC_PLL_ENABLE();

  // Wait for PLL to be ready
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) {
    HAL_Delay(1);
  }

  __enable_irq();
  return;
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
