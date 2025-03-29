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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // Include if using semaphores in ISR
#include <SEGGER_SYSVIEW.h>
#include <lookBusy.h>
#include "stm32f7xx.h"

#include "font_extra.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1_PIN        0  // PB0
#define LED2_PIN        7  // PB7
#define LED3_PIN        14 // PB14

//#define SHT31_ADDR 0x44 << 1 // SHT31 I2C address shifted left by 1 bit
//#define CMD_MEASURE_TEMP 0x2C06 // Command to measure temperature
//#define CMD_MEASURE_HUMIDITY 0x2C10 // Command to measure humidity

#define SHT31_ADDR 0x44 << 1 // SHT31 I2C address shifted left by 1 bit
// #define CMD_MEASURE_TEMP 0x2C06 // No longer needed
// #define CMD_MEASURE_HUMIDITY 0x2C10 // No longer needed
#define CMD_MEASURE_TH_HIGH_REP 0x2400 // Command for Temp & Humidity High Repeatability

// Define error codes for sensor reading
#define SHT31_READ_OK      0
#define SHT31_ERR_NOT_READY -998.0f
#define SHT31_ERR_I2C_FAIL  -999.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void SHT31_ReadTempHumidity(float* temp, float* humidity);
float roundToOneDecimal(float value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vLED1Task(void *pvParameters) {
    // Data for this task
	uint16_t lux=0;
	char msg[30];
	uint32_t timeout = 100; // Timeout for transmission

    for (;;) {
    	SEGGER_SYSVIEW_PrintfHost("vLED1Task started");
    	// Toggle LED1 (PB0)
        GPIOB->ODR ^= (1 << LED1_PIN);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 20);
        lux = HAL_ADC_GetValue(&hadc1);
        snprintf(msg, sizeof(msg), "Light: %hu \r\n", lux);
        // Transmit the data via UART2
        // HAL_UART_Transmit is a BLOCKING function. It waits until transmission is complete or timeout.
        // For a slow task like this (1000ms delay), blocking is often acceptable.
        // For faster tasks or larger data, consider HAL_UART_Transmit_IT() or HAL_UART_Transmit_DMA().
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, // Pointer to UART handle
                                                  (uint8_t *)msg,      // Pointer to data buffer
                                                  strlen(msg), // Data size (exclude null terminator)
                                                  timeout);  // Timeout duration
        if (status != HAL_OK) {
            // Handle error
        }

        // Delay for 200 ms
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void vLED2Task(void *pvParameters) {
	for (;;) {
		SEGGER_SYSVIEW_PrintfHost("vLED2Task started");
		GPIOB->ODR ^= (1 << LED2_PIN); // Toggle LED2 (PB7)
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void vLED3Task(void *pvParameters) {
	// Increased buffer size slightly just in case formatting is longer
	char msg[40];
	float temperature = 0.0f, humidity = 0.0f; // Initialize variables
	uint16_t lux=0;
	uint32_t timeout = 100; // Timeout for transmission

	for (;;) {
		SEGGER_SYSVIEW_PrintfHost("vLED3Task started");
		GPIOB->ODR ^= (1 << LED3_PIN); // Toggle LED3 (PB14)

        // Read temperature and humidity data from SHT31
        // Assumes SHT31_ReadTempHumidity correctly populates both values on success
        SHT31_ReadTempHumidity(&temperature, &humidity);

        // Format the message based on the read result
        if (temperature == SHT31_ERR_I2C_FAIL) {
            snprintf(msg, sizeof(msg), "I2C Error\r\n");
        } else if (temperature == SHT31_ERR_NOT_READY) {
             snprintf(msg, sizeof(msg), "SHT31 NACK\r\n");
        }
//      else if (isnan(temperature) || isinf(temperature) || isnan(humidity) || isinf(humidity)) { // Optional advanced check
//          snprintf(msg, sizeof(msg), "Sensor Math Err\r\n");
//      }
        else {
            // *** MODIFIED LINE: Format both Temperature and Humidity ***
            // Using %.1f for one decimal place, added units and spacing.
            // Note: %% is used to print a literal '%' sign.
            snprintf(msg, sizeof(msg), "T:%.1f°C H:%.1f%%\r\n", temperature, humidity);
        }

        // Transmit the formatted message or error status
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), timeout);
        // Optional: Check UART transmission status
        if (status != HAL_OK)
        {
            // Handle UART transmission error - maybe try sending a simpler error message
             // HAL_UART_Transmit(&huart2, (uint8_t*)"UART TX ERR\r\n", 13, timeout);
        }
        ssd1306_Fill(Black);
        float rounded_val = roundToOneDecimal(temperature);
        snprintf(msg, sizeof(msg), "%.1f#$ \r\n", rounded_val);
        ssd1306_SetCursor(5,0);
        ssd1306_WriteString("Temperature: ", Font_6x8, White);
        ssd1306_SetCursor(35,15);
        ssd1306_WriteString(msg, FontExtra_8x8, White);

        rounded_val = roundToOneDecimal(humidity);
        snprintf(msg, sizeof(msg), "%.1f \r\n", rounded_val);
        ssd1306_SetCursor(5,30);
        ssd1306_WriteString("Humidity: ", Font_6x8, White);
//        ssd1306_UpdateScreen();
        ssd1306_SetCursor(35,45);
        ssd1306_WriteString(msg, FontExtra_8x8, White);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 20);
        lux = HAL_ADC_GetValue(&hadc1);

        if (lux < 50){
//              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
          ssd1306_SetContrast(255);
          ssd1306_UpdateScreen();
          GPIOE->BSRR = (1UL << 0);
        }else{// Turn LED on
//              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
          ssd1306_SetContrast(100);
          ssd1306_UpdateScreen();
          GPIOE->BSRR = (1UL << (0 + 16));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void SHT31_ReadTempHumidity(float* temp, float* humidity)
{
    // Buffer to hold 6 bytes: Temp MSB, LSB, CRC, Humid MSB, LSB, CRC
    uint8_t data[6];
    uint16_t temp_raw, humidity_raw;

    // Command for single shot measurement, high repeatability (T & H)
    uint16_t cmd_measure_th = CMD_MEASURE_TH_HIGH_REP;
    uint8_t cmd_bytes[2];
    cmd_bytes[0] = (cmd_measure_th >> 8) & 0xFF;
    cmd_bytes[1] = cmd_measure_th & 0xFF;

    HAL_StatusTypeDef status; // Variable to check HAL return status

    // Initialize output pointers to error state
    *temp = SHT31_ERR_I2C_FAIL;
    *humidity = SHT31_ERR_I2C_FAIL;

    // 1. Check if device is ready (responds to its address)
    status = HAL_I2C_IsDeviceReady(&hi2c2, SHT31_ADDR, 2, 100);
    if (status != HAL_OK) {
        *temp = SHT31_ERR_NOT_READY; // Indicate device did not ACK
        *humidity = SHT31_ERR_NOT_READY;
        return; // Exit if device is not ready
    }

    // 2. Send the single measurement command (0x2400)
    status = HAL_I2C_Master_Transmit(&hi2c2, SHT31_ADDR, cmd_bytes, 2, 1000);
    if (status != HAL_OK) {
        return; // Exit on I2C transmission error
    }

    // 3. Wait for measurement (Use vTaskDelay in RTOS)
    // High repeatability measurement time is ~15ms according to datasheet
    vTaskDelay(pdMS_TO_TICKS(20)); // 20ms delay should be safe

    // 4. Read 6 bytes: Temp MSB, LSB, CRC, Humid MSB, LSB, CRC
    status = HAL_I2C_Master_Receive(&hi2c2, SHT31_ADDR, data, 6, 1000);
    if (status != HAL_OK) {
        return; // Exit on I2C receive error
    }
    // Note: CRC bytes (data[2] and data[5]) are read but not validated here

    // 5. Calculate Temperature from first 2 bytes
    temp_raw = (data[0] << 8) | data[1];
    *temp = ((float)temp_raw * 175.0f / 65535.0f) - 45.0f;

    // 6. Calculate Humidity from the next 2 bytes (bytes 3 and 4)
    humidity_raw = (data[3] << 8) | data[4];
    *humidity = ((float)humidity_raw * 100.0f / 65535.0f);

    // If execution reaches here, I2C reads were nominally OK, and
    // both temp and humidity calculations are complete.
}

float roundToOneDecimal(float value) {
    return roundf(value * 10.0f) / 10.0f;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();
  NVIC_SetPriorityGrouping( 0 ); // ensure proper priority grouping for freeRTOS
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  HAL_StatusTypeDef device_status;
  device_status = HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 2, 100); // Check if device ACKs
  if (device_status != HAL_OK) {
      // Device not found or communication error
      // Maybe blink an LED differently or halt here for debugging
      Error_Handler();
  }

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  // Configure LED pins as output (Direct Register Access)
  GPIOB->MODER &= ~((3 << (LED1_PIN * 2)) | (3 << (LED2_PIN * 2)) | (3 << (LED3_PIN * 2)));
  GPIOB->MODER |=  ((1 << (LED1_PIN * 2)) | (1 << (LED2_PIN * 2)) | (1 << (LED3_PIN * 2)));
  GPIOB->OTYPER &= ~((1 << LED1_PIN) | (1 << LED2_PIN) | (1 << LED3_PIN));
  GPIOB->OSPEEDR &= ~((3 << (LED1_PIN * 2)) | (3 << (LED2_PIN * 2)) | (3 << (LED3_PIN * 2)));
  GPIOB->PUPDR &= ~((3 << (LED1_PIN * 2)) | (3 << (LED2_PIN * 2)) | (3 << (LED3_PIN * 2)));

  // 1. Enable GPIOE Clock in RCC_AHB1ENR register
  // Make sure RCC definition is available (usually via stm32f7xx.h -> included in main.h)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // Set the GPIOE clock enable bit

  // Short delay after enabling clock (optional, sometimes good practice)
  // volatile uint32_t dummy_delay = RCC->AHB1ENR;
  // (void)dummy_delay; // Avoid unused variable warning

  // Configure PE0 (Pin 0 on GPIOE)
  // Make sure GPIOE definition is available (usually via stm32f7xx.h -> included in main.h)

  // 2. Configure Mode: General purpose output (01)
  GPIOE->MODER &= ~(GPIO_MODER_MODER0); // Clear bits 1:0 for Pin 0 (mask is 0b11 << 0)
  GPIOE->MODER |=  (GPIO_MODER_MODER0_0); // Set bit 0 to 1 (value is 0b01 << 0)

  // 3. Configure Output Type: Push-pull (0)
  GPIOE->OTYPER &= ~(GPIO_OTYPER_OT0); // Clear bit 0 for Pin 0 (mask is 0b1 << 0)

  // 4. Configure Output Speed: Low speed (00)
  GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0); // Clear bits 1:0 for Pin 0 (mask is 0b11 << 0)
                                              // 00 is the default state (Low speed) after clearing

  // 5. Configure Pull-up/Pull-down: No pull-up, no pull-down (00)
  GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPDR0); // Clear bits 1:0 for Pin 0 (mask is 0b11 << 0)
                                        // 00 is the default state (No pull) after clearing
  // Spin until the user starts the SystemView app, in Record mode
  while(SEGGER_SYSVIEW_IsStarted()==0)
  {
      lookBusy(100);
  }
  // Create tasks
  xTaskCreate(vLED1Task, "LED1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(vLED2Task, "LED2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(vLED3Task, "LED3", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

  // Example: Create semaphore if used by TIM6 ISR
  // xTim6Semaphore = xSemaphoreCreateBinary();

  // Start scheduler - Execution transfers to FreeRTOS tasks
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ssd1306_SetCursor(5,1);
	  ssd1306_WriteString("Temperature: ", Font_6x8, White);
	  ssd1306_UpdateScreen();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
