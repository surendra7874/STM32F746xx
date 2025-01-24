/*
 * M24C08.h
 *
 *  Created on: Sep 21, 2023
 *      Author: INVADL0583
 */

#ifndef INC_M24C08_H_
#define INC_M24C08_H_


#include "stdint.h"
#include "stm32f7xx_hal.h"


void M24C08_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void M24C08_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void M24C08_PageErase (uint16_t page);

void M24C08_Write_NUM (uint16_t page, uint16_t offset, float  fdata);
float M24C08_Read_NUM (uint16_t page, uint16_t offset);


#endif /* INC_M24C08_H_ */
