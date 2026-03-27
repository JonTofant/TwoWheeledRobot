/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <controler.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "math.h"
#include "string.h"
#include <stdbool.h>

// Custom headers
#include "cybergear.h"
#include "DDSM115.h"
#include "kinematics.h"
#include "system_init.h"
#include "telemetry.h"
#include "controler.h"
#include "joystick.h"
#include "StartupStrategy.h"
#include "StateEstimator.h"
#include "state_machine.h"
#include "JumpStrategy.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ESP32 buffer size (ROLL PITCH YAW)
#define PACKET_SIZE 19



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

// Transcieve header
CAN_TxHeaderTypeDef pTxHeader;

// Receive header
CAN_RxHeaderTypeDef pRxHeader;

// Filter for receiving all extended frames
CAN_FilterTypeDef sFilterConfig;



uint8_t RxSingleByte;
uint16_t angle_u;
int16_t velocityDDSM_RPM = 0;




uint8_t RS485_RxBuffer[RS485_BUFFER_SIZE];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint16_t getDMACurrentIndex(void);
volatile uint16_t rxReadIndex = 0;       // Your software read pointer






#define BNO080_PACKET_SIZE 25  // 1 SOF + 6×4-byte floats
#define UART1_RX_BUFFER_SIZE 128
static uint8_t  uart1RxBuffer[UART1_RX_BUFFER_SIZE];





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
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_SPI2_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  // Kick off DMA+Idle for USART1
  HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                             uart1RxBuffer,
                             UART1_RX_BUFFER_SIZE);




  // Enable UART DMA for uart4
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, RS485_RxBuffer, RS485_BUFFER_SIZE);


  // ------ PRECISE SAMPLE TIME MEASUREMENTS ------

  // Enable TRC (Trace control) - needed to use DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset the cycle counter
  DWT->CYCCNT = 0;

  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  //------------------------------------------------


// SEND QUERY TO DDSM115
  uint8_t cmd[10] = {0};
  cmd[0] = 0xAA;  // Motor ID
  cmd[1] = 0x64;
  cmd[2] = 0x00;  // High byte
  cmd[3] = 0x00;  // Low byte
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = compute_crc8(cmd, 9);
  // Set GPIO pin to transmit mode

  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
  HAL_Delay(10);  // Allow time for the command to be processed
  // Set GPIO pin to receive mode
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);




  // CAN
  // 1) Configure the CAN filter to accept *all* extended frames
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow  = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow  = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);



  // 2) Start CAN
  HAL_CAN_Start(&hcan1);



  // 3) Optionally enable Rx interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);





  	  // HAL delay to allow the system to stabilize
  	  HAL_Delay(3000);



  	  System_Init();

  	init_leg_state(&leg_state_rf);
  	init_leg_state(&leg_state_lf);
  	init_leg_state(&leg_state_rb);
  	init_leg_state(&leg_state_lb);


  	// Testing of inverse kinematics



		  // Enable Timer 3 interupt
		  HAL_TIM_Base_Start_IT(&htim3);
		  // Enable Timer 4 interupt
		  HAL_TIM_Base_Start_IT(&htim4);

		  // 6) (Re-)enable the line just in case
		  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		  HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);  // Start receiving




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  // ----- MAIN CONTROL LOOP -----
	  // Seperated by flags for auxiliary tasks and a state machine


	 if (isCANReady) {
		 isCANReady = false;
		 }

	 if (uart3_controller_packet_ready) {
		 process_joystick_input();
		 uart3_controller_packet_ready = 0;
	 }



	 if (isCYBERGEARReady){

		 if (isFallen())
		 {
			 if (startPressed == 1) isStartupStrategy = true;
		 }else{
			 if (xPressed == 1) isJumpStrategy = true;
		 }



		 if(isStartupStrategy)
		 {
			 startup_strategy_control();
		 }

		 if(isFallen()){
			 DDSM115setCurrent(0x10, 0);
			 HAL_Delay(2);
			 DDSM115setCurrent(0x11, 0);

		 }

		 if(isJumpStrategy)
		 {
			 jump_strategy_control();
		 }




		 	// ROBOT IS STANDING AND OPERATIONAL
		    if (!isFallen()) {
		    	if(isSTATIC){

		    	posture_controler();


		    	Motor_SendMITCommand(&MOTOR_CG_LF);
		    	Motor_SendMITCommand(&MOTOR_CG_LB);
		    	HAL_Delay(5);
		    	Motor_SendMITCommand(&MOTOR_CG_RF);
		    	Motor_SendMITCommand(&MOTOR_CG_RB);


				 isCYBERGEARReady = false;



		    	}
		    	// THE ROBOT IS IN LOCOMOTION STATE
		    	else if(isLOCOMOTION){
		    		// For now the same thing happens as in static

			    	posture_controler();


			    	Motor_SendMITCommand(&MOTOR_CG_LF);
			    	Motor_SendMITCommand(&MOTOR_CG_LB);
			    	HAL_Delay(5);
			    	Motor_SendMITCommand(&MOTOR_CG_RF);
			    	Motor_SendMITCommand(&MOTOR_CG_RB);


					 isCYBERGEARReady = false;
		    	}

		    }
		    else{

		   	 isCYBERGEARReady = false;

		    }
	 }


	 if (isDDSM115Ready){

		 if(!isFallen() || isStartupStrategy){

		 calculate_cascaded_motor_currents(desired_v_left,desired_v_right, &current_motor1_out, &current_motor2_out, &total_force_out);

		 //current_motor1_out = 0.0f;
		 //current_motor2_out = 0.0f;

		 DDSM115setCurrent(0x10, current_motor2_out);
		 HAL_Delay(2);
		 DDSM115setCurrent(0x11, current_motor1_out);
		 }
		 else{}

		 isDDSM115Ready = false;
	 }
	 if (isTELEMETRYReady){
		 Send_Telemetry(&huart3);
	 }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 3000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MRF_RESET_Pin|SPI2_CS_MRF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNO080_RST_GPIO_Port, BNO080_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_PS0_GPIO_Port, GYRO_PS0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MRF_RESET_Pin SPI2_CS_MRF_Pin GYRO_PS0_Pin */
  GPIO_InitStruct.Pin = MRF_RESET_Pin|SPI2_CS_MRF_Pin|GYRO_PS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO080_INT_Pin */
  GPIO_InitStruct.Pin = BNO080_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO080_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DIR_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO080_RST_Pin */
  GPIO_InitStruct.Pin = BNO080_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO080_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MRF_INT_Pin */
  GPIO_InitStruct.Pin = MRF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MRF_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, CAN_received_data);
    if (status != HAL_OK)
    {
        // Error reading the message
        return;
    }

    uint32_t extId = pRxHeader.ExtId;
    uint8_t type   = (extId >> 24) & 0x1F;   // bits28..24
    uint8_t motorID= (extId >> 8) & 0xFFFF;  // Correct: Extracts bits 23–8
				   // bits7..0




    // Check what type we received
    if (type == 2) {  // Feedback message from motor


           // Extract current angle from Bytes 0-1
           uint16_t angle_raw = (CAN_received_data[0] << 8) | CAN_received_data[1];
           float angle = ((float)angle_raw / 65535.0f) * (ANGLE_MAX - ANGLE_MIN) + ANGLE_MIN + M_PI_2 - 10*M_PI/180.0f;
           // Convert to degrees
           //angle = angle * 180.0f / 3.14159265359f;


           // Extract angular velocity from Bytes 2-3
           uint16_t velocity_raw = (CAN_received_data[2] << 8) | CAN_received_data[3];
           float velocity = ((float)velocity_raw / 65535.0f) * (-30.0f - 30.0f) + 30.0f; // -30 to 30 rad/s

           // Extract torque from Bytes 4-5
           uint16_t torque_raw = (CAN_received_data[4] << 8) | CAN_received_data[5];
           float torque = ((float)torque_raw / 65535.0f) * (-12.0f - 12.0f) + 12.0f; // -12Nm to 12Nm

           // Extract temperature from Bytes 6-7
           uint16_t temp_raw = (CAN_received_data[6] << 8) | CAN_received_data[7];
           float temperature = (float)temp_raw / 10.0f; // Temp in Celsius

           // Find matching motor in motorList and update its values
           for (int i = 0; i < MAX_MOTORS; i++) {
               if (CyberGearMotorList[i].motorID == motorID) {
            	   CyberGearMotorList[i].angle = angle;
            	   CyberGearMotorList[i].velocity = velocity;
            	   CyberGearMotorList[i].torque = torque;
            	   CyberGearMotorList[i].temperature = temperature;
            	   //CyberGearMotorList[i].errorFlag = false;
            	   CyberGearMotorList[i].update_flag = true;


                   break;  // No need to check further once a match is found
               }
           }
           // Call watchdog function
           //watchdog(&CyberGearMotorList, &DDSM115MotorList, &LegGeometryList, &sampleTime);


       }
    else if (type == 17){




    }
    else if (type == 21)
    {
        // This is typically the fault/error frame
        // Byte0..3 might be a fault code


        // Optionally parse the fault code in received_data[0..3]
        // For example:
        /*uint32_t fault = (CAN_received_data[0])
                       | (CAN_received_data[1] << 8)
                       | (CAN_received_data[2] << 16)
                       | (CAN_received_data[3] << 24);*/
    }
    else if (type == 0)
    {
        // Possibly a "Get Device ID" response
        // The data bytes may contain the 64-bit unique ID
    }
    else
    {
        // Some other type

    }
}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){

	}
	if(huart->Instance == UART4){
		//HAL_UART_Transmit(&huart2, &RxSingleByte, 1, 10);

	}

	 if (huart->Instance == USART3) {
	        if (uart3_controller_index == 0) {
	            if (uart3_controller_byte == 0xAA) {
	                uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;
	            }
	            // Else ignore random bytes
	        } else {
	            uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;
	            if (uart3_controller_index >= UART3_CONTROLLER_PACKET_LEN) {
	                uart3_controller_index = 0;
	                uart3_controller_packet_ready = 1;
	            }
	        }
	        HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);  // Re-arm
	    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	   if (huart->Instance == UART5)
	    {
	        uint8_t motor_id = RS485_RxBuffer[0];

	        for (int i = 0; i < MAX_MOTORS_DDSM115; i++) {
	            if (DDSM115MotorList[i].motorID == motor_id) {
	            	update_ddsm115_state(&DDSM115MotorList[i], RS485_RxBuffer, 0.0505f);
	                break;
	            }
	        }

	        // Restart reception for next frame
	        HAL_UARTEx_ReceiveToIdle_IT(&huart5, RS485_RxBuffer, RS485_BUFFER_SIZE);
	    }
    if (huart->Instance == USART1)
    {
        // Size = number of bytes received into uart1RxBuffer[]
        uint16_t bytes = Size;
        uint16_t idx   = 0;

        // Process as many full packets as we have
        while (idx + PACKET_SIZE <= bytes)
        {
            if (uart1RxBuffer[idx] == 0xAA)
            {
                // Sync on first packet only
                if (!uartSynced)
                {
                    uartSynced = true;
                    // shift this good packet to buffer start if desired:
                    memmove(uart1RxBuffer,
                            uart1RxBuffer + idx,
                            PACKET_SIZE);
                    idx = 0;       // start parsing from buffer[0]
                    bytes = PACKET_SIZE;
                }

                // Parse the six floats (little-endian) at idx+1..idx+24
                float *f   = (float*)(uart1RxBuffer + idx + 1);
                yaw_esp32   = f[0];
                pitch_esp32 = f[1];
                roll_esp32  = f[2];
                gx_esp32    = f[3];
                gy_esp32    = f[4];
                gz_esp32    = f[5];

                // Advance past this packet
                idx += PACKET_SIZE;
            }
            else
            {
                // No SOF here, skip one byte
                idx++;
            }
        }



        // re-arm for next Idle
        HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                   uart1RxBuffer,
                                   UART1_RX_BUFFER_SIZE);
    }





}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		// Every 10ms (100Hz)
		// Set the flag for CAN transmission
		//isCANReady = true;
	}
	if (htim->Instance == TIM4)
	{
		isDDSM115Ready = true;
		isCYBERGEARReady = true;
		isTELEMETRYReady = true;

	}
}











/**
  * @brief Updates continuous base ANGLE (phi) and ANGULAR VELOCITY (phi_dot)
  *        AND linear position (x) and linear velocity (x_dot) based on DDSM115 feedback.
  *        Handles position wrap-around. Only processes data from motor 0x10.
  * @param Buffer: Pointer to the received RS485 data buffer.
  * @param current_phi_rad: Pointer to global variable storing continuous angular position (rad).
  * @param current_phi_dot_rad_s: Pointer to global variable storing angular velocity (rad/s).
  * @param current_x: Pointer to global variable storing linear position (m).
  * @param current_x_dot: Pointer to global variable storing linear velocity (m/s).
  * @retval None (Results are stored in the global variables via pointers).
  */









void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is from our telemetry UART.
    if (huart->Instance == USART3) {
        // The transmission is complete, so we set the flag to true,
        // allowing the next call to Send_Telemetry() to proceed.
        Telemetry_UART_TxCpltCallback(huart);
    }
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
