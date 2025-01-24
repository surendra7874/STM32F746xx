/*
 * ad7779.c
 *
 *  Created on: Sep 6, 2023
 *      Author: INVADL0583
 */

#include "AD7779.h"
#include  "main.h"


SPI_HandleTypeDef hspi2;

uint8_t  receiveData[2];    // RESET value of the register
uint8_t  modifyValue1[2];   //Value of the register after modification

void readRegister(uint8_t regAdd)
{
	  uint8_t readCommand = regAdd | 0x80;      //Making the MSB bit to 1 for reading the register
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);     //CS pin Low
	 HAL_SPI_Transmit(&hspi2, &readCommand, 1, 10);    // Receiving the RESET value of the register
	 HAL_SPI_Receive(&hspi2, &receiveData, 2, 10);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // CS pin High
	  //return receiveData;
}

void modifyRegister(uint8_t modifyRegAdd, uint8_t value)
{
	 // uint8_t   value = 0x90;   //value we want to modify in the register
	  /*Making the MSB bit to zero after writing and attaching the value to the Address  */
	  uint8_t  modifyAdd[2];
	  modifyAdd[0]	 =  ((modifyRegAdd ) & ~(0x80) );
	  modifyAdd[1]	= value;
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);     //CS pin Low
	  HAL_SPI_Transmit(&hspi2, &modifyAdd, 2, 10);        //Sending the address of the register to modify
	  HAL_SPI_Receive(&hspi2, &modifyValue1, 1, 10);         // Receiving the value of the register
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);      // CS pin High
      //return modifyValue;
}

//Reset the AD7779 using the hardware reset button.
void AD7779_hardRESET(void)
{
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET); // Hardware Reset Pin Low
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);   //Hardware Reset pin High
}

//Reset the AD7779 using the software reset
void AD7779_softRESET(void)
{
	// Enabling the bit 0 and 1 in general user configuration register to 11 to make software reset at 1st write
	modifyRegister(0x11,0x27);
 }

//Enabling the SAR mode of operation
/*void modeSAR(void)
{
       uint8_t  config1 = readRegister(0x12);          // Reading the register value of general user config 2
       uint8_t  config2 = readRegister(0x13);          // Reading the register value of general user config 3
       config1 |= 0x20;            // making bit 5 to SET
       config2 |= 0x10;           // making bit 4 to SET
       modifyRegister(0x13, config1);               // Writing that bit value to the general user config 2 register
       modifyRegister(0x13, config2);              // Writing that bit value to the general user config 3 register
}*/
/*
void channelEnable(uint8_t channel)
{

}

// Reading the ADC value from the AD7779
void readADC(uint32_t channelADD)
{

}
*/

