/*
 * M24C08.c
 *
 *  Created on: Sep 22, 2023
 *      Author: INVADL0583
 */

#ifndef INC_M24C08_C_
#define INC_M24C08_C_

#include "main.h"



#define  DEVICE_ID               0xA0              // Device ID for M24C08
#define  AP_DEVICE_ID          0xA8             //Device ID of Application NVRAM(M24C08)
#define  PAGE_SIZE               16                 //Size of the Page.
#define  WRITE_DELAY           5                  // Time required to write the data


uint8_t bytestoWrite(uint8_t size, uint8_t byte);

//Read and Write functions for the Factory NVRAM

void M24C08_WritePage(uint8_t pageNo, uint8_t byte, uint8_t *dataW, uint16_t size);
void M24C08_ReadPage(uint8_t pageNo, uint8_t byte, uint8_t *dataR, uint16_t size);

//Read and Write functions for the Application NVRAM

void AP_M24C08_WritePage(uint8_t pageNo, uint8_t byte, uint8_t *dataW, uint16_t size);
void AP_M24C08_ReadPage(uint8_t pageNo, uint8_t byte, uint8_t *dataR, uint16_t size);

void WriteFloat(uint8_t page, uint8_t start_byte,float data);
void ReadFloat(uint8_t page, uint8_t start_byte);

void GENERIC_M24C08_ReadPage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataR, uint16_t size, uint8_t deviceID);
void GENERIC_M24C08_WritePage(uint8_t pageNo, uint8_t start_byte, uint8_t *dataW, uint16_t size, uint8_t deviceID);


void readTemp(uint8_t  slaveAdd, uint8_t pointerAdd);

#endif /* INC_M24C08_C_ */
