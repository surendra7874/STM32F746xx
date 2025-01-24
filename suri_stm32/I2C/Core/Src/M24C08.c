/*
 * M24C08.c
 *
 *  Created on: Sep 22, 2023
 *      Author: INVADL0583
 */
#include  "stm32f7xx.h"
#include  "M24C08.h"
#include  "stdlib.h"


extern I2C_HandleTypeDef hi2c1;

uint16_t temp =0;
uint8_t bytes_temp[4];
/*
 * Function to read or write the start_bytes that are left in that particular page.
 */
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
/*
 * @Function is used to write the data into the memory based on the size we required.
 * @pageNo is from 0 to 63
 * @start_byte is from 0 to 15
 */
/*

void M24C08_WritePage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataW, uint16_t size)
{
	uint8_t startPage = pageNo;         //Starting page to write the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to write the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to write the data
	uint8_t pos = 0;            //position of the data to write

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE) +start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDw = DEVICE_ID | ((temp >> 7) & (0x06));       //Device address of the EEPROM along with A9 and A8 bits.
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to write the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to write.

		 HAL_I2C_Mem_Write(&hi2c1, deviceIDw, memAdd, 1, &dataW[pos], bytesRemaining, 1000);

		 startPage += 1;          //Increment the page after completion of writing the data into present page.
		 start_byte = 0;                   // Making to zero so it start  writing the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are written in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to write data to the next page

		 HAL_Delay(80);       //Delay the process to get the writing value update
	}
}
*/
/*
 * @Function is used to read the data from the memory based on the size we required.
 * @pageNo is from 0 to 63
 * @start_byte is from 0 to 15
 */
/*
void  M24C08_ReadPage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataR, uint16_t size)
{
	uint8_t startPage = pageNo;         //Starting page to read the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to read the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to read the data
	uint8_t pos = 0;            //position of the data to read

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE)+start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDr = DEVICE_ID | ((temp >> 7) & (0x07));       //Device address of the EEPROM along with A9 and A8 bits and read bit
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to read the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to read.

		 HAL_I2C_Mem_Read(&hi2c1, deviceIDr, memAdd, 1, &dataR[pos], bytesRemaining, 1000);

		 startPage += 1;          //Increment the page after completion of reading the data into present page.
		 start_byte = 0;                   // Making to zero so it start  reading the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are read in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to read data to the next page
	}
}
*/
/******************************** APPLICATION NVRAM  *************************************************************/
/*
 * @Function is used to write the data into the memory based on the size we required.(For Application NVRAM)
 * @pageNo is from 0 to 63
 * @start_byte is from 0 to 15
 */
/*
void AP_M24C08_WritePage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataW, uint16_t size)
{
	uint8_t startPage = pageNo;         //Starting page to write the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to write the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to write the data
	uint8_t pos = 0;            //position of the data to write

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE) +start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDw = AP_DEVICE_ID | ((temp >> 7) & (0x06));       //Device address of the EEPROM along with A9 and A8 bits.
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to write the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to write.

		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
		 HAL_I2C_Mem_Write(&hi2c1, deviceIDw, memAdd, 1, &dataW[pos], bytesRemaining, 1000);
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);

		 startPage += 1;          //Increment the page after completion of writing the data into present page.
		 start_byte = 0;                   // Making to zero so it start  writing the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are written in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to write data to the next page

		 HAL_Delay(80);       //Delay the process to get the writing value update
	}
}*/
/*
void AP_M24C08_ReadPage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataR, uint16_t size)
{
	uint8_t startPage = pageNo;         //Starting page to read the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to read the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to read the data
	uint8_t pos = 0;            //position of the data to read

	for(int i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE)+start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDr = AP_DEVICE_ID | ((temp >> 7) & (0x07));       //Device address of the EEPROM along with A9 and A8 bits and read bit
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to read the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to read.

		 HAL_I2C_Mem_Read(&hi2c1, deviceIDr, memAdd, 1, &dataR[pos], bytesRemaining, 1000);

		 startPage += 1;          //Increment the page after completion of reading the data into present page.
		 start_byte = 0;                   // Making to zero so it start  reading the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are read in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to read data to the next page
	}
}
/*
/***************************** FLOAT TO BYTE AND VICE VERSA CONVERSION ************************************/

/*
void float2Bytes(uint8_t * FtoN_bytes_temp,float float_variable)
{
    union {
      float value;
      uint8_t bytes[4];
    } ftb;

    ftb.value = float_variable;

    for (uint8_t i = 0; i < 4; i++)
    {
      FtoN_bytes_temp[i] = ftb.bytes[i];
    }
}

float Bytes2float(uint8_t * FtoN_bytes_temp)
{
    union {
      float value;
      uint8_t bytes[4];
    } ftb;

    for (uint8_t i = 0; i < 4; i++)
    {
       ftb.bytes[i] =  FtoN_bytes_temp[i];
    }
    float float_variable = ftb.value;
    return float_variable;
}

void WriteFloat(uint8_t page, uint8_t start_byte,float data)
{
	float2Bytes(bytes_temp, data);

	M24C08_WritePage(page,start_byte,bytes_temp,4);
}

void ReadFloat(uint8_t page, uint8_t start_byte)
{
	uint8_t buffer[4];

	M24C08_ReadPage(page, start_byte, &buffer, 4);

	return (Bytes2float(buffer));

}
*/


void GENERIC_M24C08_WritePage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataW, uint16_t size, uint8_t deviceID)
{
	uint8_t startPage = pageNo;         //Starting page to write the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to write the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to write the data
	uint8_t pos = 0;            //position of the data to write

	for(uint8_t i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE) +start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDw = deviceID | ((temp >> 7) & (0x06));       //Device address of the EEPROM along with A9 and A8 bits.
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to write the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to write.

		 if(deviceID == AP_DEVICE_ID)       // Checking the device ID for Application NVRAM
		 {
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);                   //  enable the write operation
		 HAL_I2C_Mem_Write(&hi2c1, deviceIDw, memAdd, 1, &dataW[pos], bytesRemaining, 1000);
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);                   //  disable the write operation
		 }
		 else
		 {
			 HAL_I2C_Mem_Write(&hi2c1, deviceIDw, memAdd, 1, &dataW[pos], bytesRemaining, 1000);
		 }
		 startPage += 1;          //Increment the page after completion of writing the data into present page.
		 start_byte = 0;                   // Making to zero so it start  writing the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are written in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to write data to the next page

		 HAL_Delay(5);       //Delay the process to get the writing value update
	}
}


void GENERIC_M24C08_ReadPage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataR, uint16_t size, uint8_t deviceID)
{
	uint8_t startPage = pageNo;         //Starting page to read the data
	uint8_t endPage = startPage + ((size+start_byte)/PAGE_SIZE);          //end page to read the data

	uint8_t numofPages = (endPage - startPage)+1;        //Number of pages to read the data
	uint8_t pos = 0;            //position of the data to read

	for(uint8_t i=0; i< numofPages; i++)
	{
		 temp = (startPage * PAGE_SIZE)+start_byte;  // checking the starting address of the start_byte to start.
		 uint8_t  deviceIDr = deviceID | ((temp >> 7) & (0x07));       //Device address of the EEPROM along with A9 and A8 bits and read bit
		 uint8_t  memAdd = (temp & 0xFF);                                       //memory address to where we want to read the data
		 uint8_t bytesRemaining = bytestoWrite(size,start_byte);             //Number of start_byte are pending in that page to read.

		 HAL_I2C_Mem_Read(&hi2c1, deviceIDr, memAdd, 1, &dataR[pos], bytesRemaining, 1000);

		 startPage += 1;          //Increment the page after completion of reading the data into present page.
		 start_byte = 0;                   // Making to zero so it start  reading the data from 1st.
		 size = size - bytesRemaining;    //Subtracting the Number of start_bytes from the size which are read in the previous page.
		 pos += bytesRemaining;           // position of the data from where we want to read data to the next page
	}
}


/************************************************************************************
 * **********************************************************************************
                              LM73 Temperature sensor
 * ***********************************************************************************
 *************************************************************************************/

void readTemp(uint8_t  slaveAdd, uint8_t pointerAdd)
{
	uint8_t  data[2];

    HAL_I2C_Mem_Read(&hi2c1, slaveAdd, pointerAdd, 1, data, 2, 20);

    uint16_t temp1 = (data[1] << 8) | data[0];
    uint16_t temp2 = (((~temp1) | 0x01) >> 5);
    uint16_t finalValue
    return finalValue;
}









