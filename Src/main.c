/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "blackImageSmall.h"
//#include "redImageSmall.h"
//#include "blackImageLarge.h"
#include "redImageLarge.h"
#include "font.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void EPD_init(void);
void PIC_display(const unsigned char* picData_old,const unsigned char* picData_new);
//void PIC_display_black(const unsigned char* picData_old);
//void PIC_display_red(const unsigned char* picData_new);
void EPD_sleep(void);
void EPD_refresh(void);
void PIC_display_Clean(void);
void PIC_display_Clean_Big(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char frameBuffer[15000];


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
  MX_SPI1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  char message[500] = "";

  int characterRowCounter = 0; //keeps track of which row to write pixles for.
  //Sometimes, it will increment when it's still rendering the same row of the character because the character is 2 bytes wide

  int stringIndex = 0; //keeps track of which character in the string we are at
  int asciiValueOfCharacter = 0; //this variables holds the ascii value of a character

  int numberOfCharactersPerRow = 16; //how many characters in a row

  int offsetFromASCII0 = 32; //offset from the actual ascii table.  A is ASCII 65, but the first letter in the array.
  int characterHeight = 72;

  int date;

  unsigned int i;

  char output[1]; //this is for the spi output and is irrevelant.
  char input[500]; //this is the string that the SP inputs to
  char previousInput[500]; //this holds the previous input

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  //HAL_SPI_TransmitReceive(which spi, spi output, spi input, spi array size, maximum spi delay);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&output, (uint8_t*)&input, 300, HAL_MAX_DELAY);
	//Input is in the input variable

	//this down here is to deal with an spi issue where it remains high for a few bytes. The first byte is always a small number
	int index = 0;
	while(input[index] > 6){
		index++;
	}

	//this checks if the message is intended for this display by checking the device id in the packet.

	//THIS WILL CHANGE DEPENDING ON WHICH DISPLAY WE ARE PROGRAMMING------------------------------------------------------------------------------------------------------------------------------
	//THIS WILL CHANGE DEPENDING ON WHICH DISPLAY WE ARE PROGRAMMING------------------------------------------------------------------------------------------------------------------------------
	//THIS WILL CHANGE DEPENDING ON WHICH DISPLAY WE ARE PROGRAMMING------------------------------------------------------------------------------------------------------------------------------
	if((int)input[index] == 6){ //checks if message is for this display
		//this if statement checks if the input string is the same as the current message. If they are the same,
		//we don't do anything, if they are not, then we need to update the display again.
		if(strcmp(previousInput, input) == 0){ //the display remains the same because the input is the same as the current displayed image
			//do nothing
		} else {
			memcpy(previousInput, input, sizeof(input));
		//start by getting the message for the display driver

			//this gets the date of the event
			date = input[index + 7];
			int tens = date / 10;
			int ones = date - tens * 10;

			message[0] = tens + 48;
			message[1] = ones + 48;
			message[2] = ' ';

			int iOffset = 0; //this variable is to keep track of i getting off in the message index because there are some opperations where we don't want i to increment in the message array, but we do for the loop
			//this sees how many events there are and writes the events to the display
			if(input[index + 1] == 0){ //when there are no events
				// do nothing
			} else if(input[index + 1] == 1){ //what to do if there is one event.
				//set the for loop upper bound to 32
				for(i = 8; i < 40; i++){
					if(i == 8 || i == 9 || i == 10){
						iOffset++; //we want to add to the offset because we don't want i to increment in message but we do in the loop.
					} else {
						message[i - 5 - iOffset] = input[index + i]; //this adds the character in the event.
					}
				}
			} else if(input[index + 1] >= 2){ //what to do if there are two or more events.  It will only display 2 events.
				//set the loop upper bound to
				for(i = 8; i < 72; i++){
					if(i == 8 || i == 9 || i == 10 || i == 40 || i == 41 || i == 42){
						iOffset++; //we want to add to the offset because we don't want i to increment in message but we do in the loop.
					} else {
						message[i - 5 - iOffset] = input[index + i]; //this adds the character in the event.
					}
				}
			}


			//this fills in the rest of the message with spaces until the end
			for(int k = sizeof(message); k < 64; k++){
				message[k] = 32;
			}

			//start the display driver

			//EPD_Clean
			EPD_init(); //EPD init
			PIC_display_Clean_Big();//EPD_Clean
			EPD_refresh();//EPD_refresh
			EPD_sleep();//EPD_sleep

			//strcat(message, restOfMessage);


			for(i = 0; i < 15000; i++){
				if(i >= 7200 && i < 8200){
					if(7300 <= i && i < 7400){
						frameBuffer[i] = 0b00000000;
					} else {
						frameBuffer[i] = 0b11111111;
					}
				} else {
					asciiValueOfCharacter = message[stringIndex];

					if(asciiValueOfCharacter == 0){
						asciiValueOfCharacter += offsetFromASCII0;
					}

					frameBuffer[i] = font[asciiValueOfCharacter - offsetFromASCII0][characterRowCounter * 3];
					i++;
					frameBuffer[i] = font[asciiValueOfCharacter - offsetFromASCII0][(characterRowCounter * 3) + 1];
					i++;
					frameBuffer[i] = font[asciiValueOfCharacter - offsetFromASCII0][(characterRowCounter * 3) + 2];


					stringIndex++; //moves to the next character


					if((stringIndex) % numberOfCharactersPerRow == 0){ //we are at the end of a row
						stringIndex -= numberOfCharactersPerRow; //this goes to the beginning of the row again
						characterRowCounter++; //this goes to the next row
						i++;
						frameBuffer[i] = 0b11111111;
						i++;
						frameBuffer[i] = 0b11111111;
					}

					if(characterRowCounter == characterHeight){ //an entire row of characters is finished rendering
						//start the next row
						stringIndex += numberOfCharactersPerRow;
						//fills in the last two spaces
						characterRowCounter = 0;
					}
				}
			}
			//this resets this variables to be used again
			characterRowCounter = 0;
			stringIndex = 0;
			asciiValueOfCharacter = 0;

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			EPD_init(); //EPD init
			PIC_display(frameBuffer, redImage);//EPD_picture2
			EPD_refresh();//EPD_refresh
			EPD_sleep();//EPD_sleep


		}
	} else { //not meant for this display
		//do nothing
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RES_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RES_Pin DC_Pin */
  GPIO_InitStruct.Pin = RES_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void EPD_W21_Init(void){
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, 0);		// Module reset
	//driver_delay_xms(1000);//At least 10ms delay
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, 1);;
	//driver_delay_xms(1000);//At least 10ms delay
	HAL_Delay(1000);
}

void EPD_init(void)
{
	  uint8_t HRES=0x80;						//128
	  uint8_t VRES_byte1=0x01;			//296
	  uint8_t VRES_byte2=0x28;

		EPD_W21_Init();

		EPD_W21_WriteCMD(0x06);         //boost soft start
		EPD_W21_WriteDATA (0x17);		//A
		EPD_W21_WriteDATA (0x17);		//B
		EPD_W21_WriteDATA (0x17);		//C

		EPD_W21_WriteCMD(0x04);
		lcd_chkstatus();

		EPD_W21_WriteCMD(0x00);			//panel setting
		EPD_W21_WriteDATA(0x0f);		//LUT from OTP��128x296
	  EPD_W21_WriteDATA(0x0d);     //VCOM to 0V fast

		EPD_W21_WriteCMD(0x61);			//resolution setting
		EPD_W21_WriteDATA (HRES);
		EPD_W21_WriteDATA (VRES_byte1);
		EPD_W21_WriteDATA (VRES_byte2);

		EPD_W21_WriteCMD(0X50);			//VCOM AND DATA INTERVAL SETTING
		EPD_W21_WriteDATA(0x77);		//WBmode:VBDF 17|D7 VBDW 97 VBDB 57		WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
}

void EPD_refresh(void){
	EPD_W21_WriteCMD(0x12);			//DISPLAY REFRESH
	//driver_delay_xms(100);	        //!!!The delay here is necessary, 200uS at least!!!
	HAL_Delay(100);
	//lcd_chkstatus();                //waiting for the electronic paper IC to release the idle signal
}

void EPD_sleep(void){
	EPD_W21_WriteCMD(0X02);  	//power off
	EPD_W21_WriteCMD(0X07);  	//deep sleep
	EPD_W21_WriteDATA(0xA5);
}

/*void PIC_display_small(const unsigned char* picData_old,const unsigned char* picData_new){
	unsigned int i;
	uint8_t value = 0;
	//black display
	EPD_W21_WriteCMD(0x10);	       //Transfer old data
	for(i=0;i<4736;i++){
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}

	//red display
	EPD_W21_WriteCMD(0x13);		     //Transfer new data
	for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(*picData_new);
	  picData_new++;
	}

}*/

void PIC_display(const unsigned char* picData_old,const unsigned char* picData_new){
	unsigned int i;
	uint8_t value = 0;
	//black display
	EPD_W21_WriteCMD(0x10);	       //Transfer old data
	for(i=0;i<15000;i++){
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}

	//red display
	EPD_W21_WriteCMD(0x13);		     //Transfer new data
	for(i=0;i<15000;i++)
	{
	  EPD_W21_WriteDATA(*picData_new);
	  picData_new++;
	}

}

/*void PIC_display_big_black(const unsigned char* picData_old){
	unsigned int i;
	uint8_t value = 0;
	//black display
	EPD_W21_WriteCMD(0x10);	       //Transfer old data
	for(i=0;i<15000;i++){
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}
}*/

/*void PIC_display_small_black(const unsigned char* picData_old){
	unsigned int i;
	uint8_t value = 0;
	//black display
	EPD_W21_WriteCMD(0x10);	       //Transfer old data
	for(i=0;i<4736;i++){
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}
}*/

/*void PIC_display_big_red(const unsigned char* picData_new){
	unsigned int i;

	//red display
	EPD_W21_WriteCMD(0x13);		     //Transfer new data
	for(i=0;i<15000;i++)
	{
	  EPD_W21_WriteDATA(*picData_new);
	  picData_new++;
	}
}*/


void PIC_display_Clean(void)
{
    unsigned int i;
		EPD_W21_WriteCMD(0x10);	       //Transfer old data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(0xff);
	}

		EPD_W21_WriteCMD(0x13);		     //Transfer new data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(0xff);
	}
}

void PIC_display_Clean_Big(void){
    unsigned int i;
	EPD_W21_WriteCMD(0x10);	       //Transfer old data
	for(i=0;i<15000;i++){
	  EPD_W21_WriteDATA(0xff);
	}

	EPD_W21_WriteCMD(0x13);		     //Transfer new data
	for(i=0;i<15000;i++){
	  EPD_W21_WriteDATA(0xff);
	}
}


void lcd_chkstatus(void)
{
	unsigned char busy;
	do
	{
		EPD_W21_WriteCMD(0x71);
		busy = HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin);
		busy =!(busy & 0x01);
	}
	while(busy);
	HAL_Delay(200);
	//driver_delay_xms(200);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
