#include "main.h"
#ifndef _MCU_SPI_H_
#define _MCU_SPI_H_
//#include "stm32f10x.h"

#define EPD_W21_SPI_SPEED 0x02

#define EPD_W21_MOSI_0	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define EPD_W21_MOSI_1	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)

#define EPD_W21_CLK_0	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define EPD_W21_CLK_1	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

//CS is to GPIOA12
#define EPD_W21_CS_0	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define EPD_W21_CS_1	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)

//DC is to GPIOA11
#define EPD_W21_DC_0	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
#define EPD_W21_DC_1	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)

//RST is to GPIOA10
#define EPD_W21_RST_0	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define EPD_W21_RST_1	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

//BUSY is to GPIOA9
#define isEPD_W21_BUSY  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) // for solomen solutions

extern void driver_delay_us(unsigned int xus);
extern void driver_delay_xms(unsigned long xms);

void SPI_Write(unsigned char value);
void EPD_W21_WriteDATA(unsigned char command);
void EPD_W21_WriteCMD(unsigned char command);


#endif  //#ifndef _MCU_SPI_H_

/***********************************************************
						end file
***********************************************************/
