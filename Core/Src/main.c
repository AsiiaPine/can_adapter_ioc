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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t channel; // 1 for FDCAN1, 2 for FDCAN2
} can_message_t;

typedef struct {
    uint8_t command;
    uint8_t channel;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} usb_can_frame_t;

/* USB-CAN Protocol Commands */
#define CMD_CAN_SEND      0x01
#define CMD_CAN_RECEIVE   0x02
#define CMD_CAN_STATUS    0x03
#define CMD_CAN_CONFIG    0x04
#define CMD_CAN_RESET     0x05
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUFFER_SIZE 64
#define CAN_MAX_MESSAGES 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// static uint8_t usb_rx_buffer[USB_BUFFER_SIZE];
// static uint8_t usb_tx_buffer[USB_BUFFER_SIZE];
// static uint8_t usb_rx_index = 0;
// static uint8_t usb_tx_index = 0;

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  
  // Start CAN reception
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  
  // Activate CAN RX notifications
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  // Initialize LED status
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Red LED off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   // Green LED off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);   // Blue LED off
  
  // Send initial status
  uint8_t init_msg[] = "USB-CAN Adapter Ready\r\n";
  CDC_Transmit_FS(init_msg, strlen((char*)init_msg));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Feed watchdog
    HAL_IWDG_Refresh(&hiwdg);
    
    // Update LED status
    update_led_status();
    
    // Process any pending USB data
    if (usb_rx_index > 0) {
      process_usb_command(usb_rx_buffer, usb_rx_index);
      usb_rx_index = 0;
    }
    
    // Send any queued CAN messages
    while (can_rx_queue_head != can_rx_queue_tail) {
      can_message_t *msg = &can_rx_queue[can_rx_queue_tail];
      
      usb_can_frame_t frame;
      frame.command = CMD_CAN_RECEIVE;
      frame.channel = msg->channel;
      frame.id = msg->id;
      frame.dlc = msg->dlc;
      memcpy(frame.data, msg->data, 8);
      
      send_usb_response((uint8_t*)&frame, sizeof(frame));
      
      can_rx_queue_tail = (can_rx_queue_tail + 1) % CAN_MAX_MESSAGES;
    }
    
    HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
            msg.id = frame->id;
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
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = msg->dlc;
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
        }
    }
}

static void send_usb_response(uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS(data, len);
}

static void update_led_status(void)
{
    static uint32_t led_timer = 0;
    static uint8_t led_state = 0;
    
    led_timer++;
    if (led_timer >= 500) { // 500ms
        led_timer = 0;
        led_state = !led_state;
        
        // Green LED: USB connected
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        
        // Blue LED: CAN activity
        if (can1_rx_count > 0 || can2_rx_count > 0 || can1_tx_count > 0 || can2_tx_count > 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
        }
    }
}

static void toggle_led(uint16_t pin)
{
    HAL_GPIO_TogglePin(GPIOC, pin);
}

/* CAN Interrupt Callbacks */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if(FDCAN1 == hfdcan->Instance){
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
      /* Retrieve Rx messages from RX FIFO0 */
      if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        receive_can_message(hfdcan, FDCAN_RX_FIFO0);
      }
      }
    return;
  }
  if (FDCAN2 == hfdcan->Instance){
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {

      if (RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        receive_can_message(hfdcan, FDCAN_RX_FIFO0);
      }
      }
    return;
  }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    // Optional: Handle transmission events
    UNUSED(hfdcan);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    // Handle CAN errors
    if (hfdcan == &hfdcan1) {
        // Toggle red LED to indicate CAN1 error
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    } else if (hfdcan == &hfdcan2) {
        // Toggle red LED to indicate CAN2 error
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
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
