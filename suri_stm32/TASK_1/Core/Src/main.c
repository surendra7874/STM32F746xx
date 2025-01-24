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

#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PAGE_SIZE    16
#define DEVICE_ID_W   0xA8
#define DEVICE_ID_R   0xA9
#define START_PAGE_ON  0x00
#define START_PAGE_OFF  0x11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char RxData[5] = {0};
char RxBuffer[5] = {0};
uint8_t rxIndex = 0;
uint8_t countOn = 0;
uint8_t countOff = 0;
char *onCond ="ON";
char *offCond ="OFF";
char ItoA[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RxBuffer[rxIndex] = RxData[0];  //Storing the data sent from the user.
	rxIndex++;    //Incrementing the index value in the buffer
	if(rxIndex > 3)
	{
		memset(RxBuffer,'\0',5);  //Clearing the buffer
		rxIndex = 0;    //making the index value to zero.
	}
	HAL_UART_Receive_IT(&huart6, &RxData, 1);
}


uint8_t bytestoWrite(uint8_t size, uint8_t start_byte)
{
	 /* If the start_bytes+size is less than PAGE_SIZE then it will return the size which is entered by us manually */
	if((size+start_byte) < PAGE_SIZE)
	{
		return size;
	}
	 /*If the start_bytes+size is greater than PAGE_SIZE then it will return the "PAGE_SIZE-start_bytes" (remaining start_bytes left in that page).*/
	else
	{
		return PAGE_SIZE-start_byte;
	}
}

void AP_M24C08_WritePage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataW, uint16_t size)
{
	uint16_t temp;
	uint8_t startPage = pageNo;         //Starting page to write the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to write the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to write the data
	uint8_t pos = 0;            //position of the data to write

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE) +start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDw = DEVICE_ID_W | ((temp >> 7) & (0x06));       //Device address of the EEPROM along with A9 and A8 bits.
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to write the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to write.

		 HAL_GPIO_WritePin(WRITEPROTECT_GPIO_Port, WRITEPROTECT_Pin, GPIO_PIN_RESET);
		 HAL_I2C_Mem_Write(&hi2c1, deviceIDw, memAdd, 1, &dataW[pos], bytesRemaining, 1000);
		 HAL_GPIO_WritePin(WRITEPROTECT_GPIO_Port, WRITEPROTECT_Pin, GPIO_PIN_SET);

		 startPage += 1;          //Increment the page after completion of writing the data into present page.
		 start_byte = 0;                   // Making to zero so it start  writing the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are written in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to write data to the next page

		 HAL_Delay(80);       //Delay the process to get the writing value update
	}
}

void AP_M24C08_ReadPage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataR, uint16_t size)
{
	uint16_t temp;
	uint8_t startPage = pageNo;         //Starting page to read the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to read the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to read the data
	uint8_t pos = 0;            //position of the data to read

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE)+start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDr = DEVICE_ID_W | ((temp >> 7) & (0x07));       //Device address of the EEPROM along with A9 and A8 bits and read bit
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to read the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to read.

		 HAL_I2C_Mem_Read(&hi2c1, deviceIDr, memAdd, 1, &dataR[pos], bytesRemaining, 1000);

		// HAL_UART_Transmit(&huart6, dataR, 1, 100);

		 startPage += 1;          //Increment the page after completion of reading the data into present page.
		 start_byte = 0;                   // Making to zero so it start  reading the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are read in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to read data to the next page
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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /*char sendChar = 1;
  char receiveChar ; */

  uint8_t mem_ON;
  uint8_t mem_OFF;

  char onBuff[2];
  char offBuff[2];

  AP_M24C08_ReadPage(0,0,&mem_ON,1);   //Reading the On Count value from the NVRAM.
  sprintf(onBuff, "%d",mem_ON);        //Converting the integer value into char
  HAL_UART_Transmit_IT(&huart6, &onBuff, 2);  //Transmitting the on count value.

  AP_M24C08_ReadPage(1,0,&mem_OFF,1);  //Reading the Off Count value from the NVRAM.
  sprintf(offBuff, "%d",mem_OFF);      //Converting the integer value into char
  HAL_UART_Transmit_IT(&huart6, &offBuff,2);  //Transmitting the off count value.


  //Receiving the input from the user.
  HAL_UART_Receive_IT(&huart6, &RxData, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        //Checking the condition for ON
		if(!strcmp(RxBuffer,onCond))
		{
		  HAL_UART_Transmit(&huart6, "LED ON", 6, 100);
		  HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_RESET);
		  countOn++;   //Incrementing the value of LED ON count
		  AP_M24C08_WritePage(0,0,&countOn,1);  //Writing the ON count value to NVRAM.
		  rxIndex = 0;     //Making the index value to zero.
		  memset(RxBuffer,'\0',5);  //Clearing the RxBuffer.
		}
		//Checking the condition for OFF
		else if(!strcmp(RxBuffer,offCond))
		{
		    HAL_UART_Transmit(&huart6, "LED OFF", 7, 100);
		    HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_SET);
			countOff++;  //Incrementing the value of LED OFF count
			AP_M24C08_WritePage(1,0,&countOff,1);   //Writing the OFF count value to NVRAM.
		    rxIndex = 0;    //Making the index value to zero.
			memset(RxBuffer,'\0',5);  //Clearing the RxBuffer.
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WRITEPROTECT_GPIO_Port, WRITEPROTECT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : WRITEPROTECT_Pin */
  GPIO_InitStruct.Pin = WRITEPROTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(WRITEPROTECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_GREEN_Pin */
  GPIO_InitStruct.Pin = LED1_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED1_GREEN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*void bytesLeft( uint8_t startByte, uint8_t size)
{
	if((startByte + size) < 16)
		return size;
	else
		return PAGESIZE - startByte;
}

void writeData( uint8_t startPage, uint8_t startByte,uint8_t *dataW, uint8_t size)
{
	uint16_t  endPages =  ((startPage * PAGESIZE) + (startByte + size)) / PAGESIZE;
	uint8_t  numPages = (endPage - startPage) + 1;
	uint8_t pos = 0;


	while(size > 0)
	{
		uint8_t BytesLeftInPage = bytesLeft(startByte,size);
		uint16_t temp = (startPage * PAGESIZE) + startByte;
		uint8_t memADD = temp & 0xFF;
		uint8_t temp1 = (temp >> 7) & 0x06;
		uint8_t slaveADD = DEVICE_ID_W | temp1;

		HAL_I2C_Mem_Write(&hi2c1, slaveADD, memADD, 1, &dataW[pos], BytesLeftInPage, 100);

		startPage += 1;
		startByte = 0;
		size = size - bytesLeft;
		pos += BytesLeftInPage;
	}
}

void readData( uint8_t startPage, uint8_t startByte,uint8_t *dataR, uint8_t siz)
{
	uint16_t  endPages =  ((startPage * PAGESIZE) + (startByte + size)) / PAGESIZE;
	uint8_t  numPages = (endPage - startPage) + 1;
	uint8_t pos = 0;


	while(size > 0)
	{
		uint8_t BytesLeftInPage = bytesLeft(startByte,size);
		uint16_t temp = (startPage * PAGESIZE) + startByte;
		uint8_t memADD = temp & 0xFF;
		uint8_t temp1 = (temp >> 7) & 0x06;
		uint8_t slaveADD = DEVICE_ID_R | temp1;

		HAL_I2C_Mem_Write(&hi2c1, slaveADD, memADD, 1, &dataR[pos], BytesLeftInPage, 100);

		startPage += 1;
		startByte = 0;
		size = size - bytesLeft;
		pos += BytesLeftInPage;
	}
}*/

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
