/*
 * AD7779.h
 *
 *  Created on: Sep 6, 2023
 *      Author: INVADL0583
 */

#ifndef INC_AD7779_H_
#define INC_AD7779_H_

#include "main.h"


#define  AD7779_REG_GENERAL_USER_CONFIG_1              0x011             // Address of the General user register 1
#define  AD7779_REG_GENERAL_USER_CONFIG_2              0x012             // Address of the General user register 2
#define  AD7779_REG_GENERAL_USER_CONFIG_3              0x013             // Address of the General user register 3
#define  AD7779_REG_DATA_OUTPUT_FORMAT                  0x014             // Address of the data output format
#define  AD7779_REG_GPIO_CONFIG                                     0x017             //Address  of the GPIO configuration register
#define  AD7779_REG_GPIO_DATA                                          0x018             // Address of the GPIO data register

//AD7779 General User Register configuration 1 details

#define   AD777_9ALL_CH_DIS_MCLK_EN       (1 << 7)                      //setting the DCLK to continue toggling
#define   AD777_POWER_MODE                     (1 << 6)                       //High Resolution Power Mode
#define   AD777_PDB_VCM                              (1 << 5)                       // Power down VCM BUffer
#define   AD777_PDB_REFOUT_BUF                (1 << 4)                      //Power down internal reference output buffer
#define   AD777_PDB_SAR                                (1 << 3)                      //Power down SAR
#define   AD777_PDB_RC_OSC                        (1 << 2)                       // Power down signal for internal oscillator
#define   AD777_SOFT_RESET(x)                     (((x) & 0x3)  << 0)       //Soft reset


//AD7779 General User Register configuration 2 details

#define  AD7779_SAR_DIAG_MODE_EN            (1 << 5)                           //Sets the SPI interface to read back SAR result on SDO
#define  AD7779_SDO_DRIVE_STR(x)                (((x) & 0x3) << 4)           //SDO driver strength
#define  AD7779_DOUT_DRIVE_STR(x)              (((x) & 0x3) << 2)           //DOUT Drive strength
#define  AD7779_SPI_SYNC                                 (1 << 0)                           //SYNC ADNed with value on the START PIN

//AD7779 General User Register configuration 3 details

#define  AD7779_CONVST_DEGLITCH_DIS(x)         (((x) & 0x3) << 7)              //Disable deglitching   of CONVST pin
#define  AD7779_SPI_SLAVE_MODE_EN                 (1<< 4)                               //Enable SPI slave Mode to read the back ADC on SDO
#define  AD7779_CLK_QUAL_DIS                             (1 << 0)                            //If user to require to use an MCLK signal > 256KHz

// Ad7779  Data Output Format Register

#define  AD7779_DOUT_FORMAT(x)                        (((x)&0x3) << 7)           //data out format
#define  AD7779_DOUT_HEADER_FORMAT (x)       (1 << 5)                        //Dout header format
#define  AD7779_DCLK_CLK_DIV(x)                         (((x)&0x7) << 3)          //Divide MCLK



// AD7779 configuration enumeration

typedef enum
{
	AD7779_INT_REG,
	AD7779_SD_CONV,
	AD7779_SAR_CONV,
} ad7779_spi_op_mode;

typedef enum
{
	AD7779_ENABLE,
	AD7779_DISABLE,
} ad7779_state;

typedef enum
{
	AD7779_DOUT_FORMAT_4LINES,
	AD7779_DOUT_FORMAT_2LINES,
	AD7779_DOUT_FORMAT_1LINE
}ad7779_dout_format;

typedef enum
{
	AD7779_HEADER_STATUS,
	AD7779_HEADER_CRC
}ad7779_dout_header_format;

typedef enum
{
	AD7779_DCLK_DIV_1,
	AD7779_DCLK_DIV_2,
	AD7779_DCLK_DIV_4,
	AD7779_DCLK_DIV_8,
	AD7779_DCLK_DIV_16,
	AD7779_DCLK_DIV_32,
	AD7779_DCLK_DIV_64,
	AD7779_DCLK_DIV_128,
} ad7779_dclk_div;

typedef enum
{
	AD7779_HIGH_RES,
	AD7779_LOW_PWR,
} ad7779_pwr_mode;

typedef enum
{
	AD7779_EXT_REF,
	AD7779_INT_REF,
} ad7779_ref_type;

// Functions for general configurations

void readRegister(uint8_t regAdd);
void modifyRegister(uint8_t modifyRegAdd, uint8_t value);
void AD7779_hardRESET(void);
void AD7779_softRESET(void);
void modeSAR(void);








#endif /* INC_AD7779_H_ */
