/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DATA_PORT  GPIOA
#define E_PIN      EN_Pin
#define RS_PIN     RS_Pin
#define RW_PIN     RW_Pin

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char  RxData[9] = {0};
char RxBuffer[32] = {0};
uint8_t rxIndex = 0;
uint32_t uartTick = 0;

//Variables for into to char converter
int index = 0;
char int_Ch[20];

void LCD_Cmd(uint8_t command)
{
    // Set RS and RW pins for command mode and write
    HAL_GPIO_WritePin(DATA_PORT, RS_PIN, GPIO_PIN_RESET); //Command Mode
    HAL_GPIO_WritePin(DATA_PORT, RW_PIN, GPIO_PIN_RESET); //Write Operation

    // Send the command to the data port
    HAL_GPIO_WritePin(DATA_PORT, 0xFF, command);

    // Toggle the Enable (E) pin to latch the command
    HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);  // Adjust this delay as needed
    HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_RESET);
}

void LCD_Init (void)
{
    // LCD initialization commands
    HAL_Delay(40);  // Wait for LCD to power up

    // Function Set (8-bit, 2 lines, 5x8 font)
    LCD_Cmd(0x38);

    // Display On/Off Control (Display on, Cursor off, Blink off)
    LCD_Cmd(0x0C);

    // Clear Display
    LCD_Cmd(0x01);

    // Entry Mode Set (Increment cursor position, no display shift)
    LCD_Cmd(0x06);
}

//Function for the LCD data to the send by the user.
void LCD_Print(char* text)
{
    // Set RS and RW pins for data mode and write
    HAL_GPIO_WritePin(DATA_PORT, RS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DATA_PORT, RW_PIN, GPIO_PIN_RESET);

    // Loop through the characters in the text
    while (*text)
    {
        // Send the character to the data port
        HAL_GPIO_WritePin(DATA_PORT, 0xFF, *text);

        // Toggle the Enable (E) pin to latch the data
        HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_SET);
        HAL_Delay(1);  // Adjust this delay as needed
        HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_RESET);

        text++;
    }

}

//Function to convert the int value to the char value

void int_to_char(int num)
{
    while(num)
    {
        int temp = num % 10;
        int_Ch[index] = temp + 48;
        index++;
        num = num / 10;
    }
    for(int i=(index -1); i >= 0; i--)
    {
        printf("%c", int_Ch[i]);
    }
}


void LCD_Print_Int(int num)
{
	char chBuffer[32];

	sprintf(chBuffer,"%d",num);

    // Set RS and RW pins for data mode and write
    HAL_GPIO_WritePin(DATA_PORT, RS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DATA_PORT, RW_PIN, GPIO_PIN_RESET);

    while (*chBuffer)
    {
        // Send the character to the data port
        HAL_GPIO_WritePin(DATA_PORT, 0xFF, *chBuffer);

        // Toggle the Enable (E) pin to latch the data
        HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_SET);
        HAL_Delay(1);  // Adjust this delay as needed
        HAL_GPIO_WritePin(DATA_PORT, E_PIN, GPIO_PIN_RESET);

    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartTick = HAL_GetTick();

	RxBuffer[rxIndex] = RxData[0];
	rxIndex++;

	//clear the data when data size is greater than 32 bytes.
	if(rxIndex >= 32)
	{
		LCD_Print("SIZE OF DATA IS");
		LCD_Cmd(0xC1);     //command to move the cursor to next line.
		LCD_Print("   LARGE   ");
		RxBuffer[32] = 0;
		rxIndex = 0;
	}

	HAL_UART_Receive_IT(&huart6,&RxData,1);  //Receiving the data from the user byte by byte.
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
  LCD_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  LCD_Print("  WELCOME  TO  ");
  LCD_Cmd(0xC1);
  LCD_Print("AMM DEMO PROJECT");
  LCD_Print_Int(123456);
  HAL_UART_Receive_IT(&huart6, RxData, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if((HAL_GetTick() - uartTick) > 10)
		{
			LCD_Print(*RxBuffer);

			memset(RxBuffer,'\0', 50);
			rxIndex = 0;
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
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |RS_Pin|D0_Pin|RW_Pin|D2_Pin
                          |D1_Pin|EN_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           RS_Pin D0_Pin RW_Pin D2_Pin
                           D1_Pin EN_Pin D3_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |RS_Pin|D0_Pin|RW_Pin|D2_Pin
                          |D1_Pin|EN_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
