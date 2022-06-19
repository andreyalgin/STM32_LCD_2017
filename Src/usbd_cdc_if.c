/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
#include "TFT.h"
#include "ctype.h"
#include "stm32f3xx_hal.h"
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */

#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64

/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

BITMAP* fontHeader;

BITMAP blocks[4] = {{582, 70, 16, 16}, {1094, 582, 16, 16}, {1606, 1094, 16, 16}, {2118, 1606, 16, 16}};

BITMAP bitmapHeader;
WAV wavHeader;

CMD cmd1;

uint32_t byteCount = 0;

uint32_t PosFrame;
uint32_t Pos;
uint16_t Type;
uint32_t Size;

uint32_t WAVStartAddress;
uint32_t WAVEndAddress;
uint8_t packetProcessed = 1;

uint8_t marioBMP[2118];

//struct CMD cmd1;
	
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */

	extern uint8_t TahomaLCD[12000];
	extern uint8_t newData;
	
	extern uint8_t TFT_busy;
	extern VerticalScrolling scrolling;
	extern RTC_TimeTypeDef RTC_Time;
	extern RTC_DateTypeDef RTC_Date;
	extern RTC_HandleTypeDef hrtc;
	extern DAC_HandleTypeDef hdac;
	extern TIM_HandleTypeDef htim7;
	
	extern uint16_t RED;
	extern uint16_t GREEN;
	extern uint16_t BLUE;
	extern uint16_t BLACK ;
	extern uint16_t YELLOW;
	extern uint16_t WHITE;

	extern  uint16_t CYAN;
	extern  uint16_t BRIGHT_RED;
	extern  uint16_t GRAY1;
	extern  uint16_t GRAY2;

	extern uint16_t LIGHTBLUE;
	
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

void DrawBlocks(uint16_t n, uint16_t posX, uint16_t posY, uint8_t a, uint8_t b)
{
	TFT_FillScreen(0, 320, 240, 0, LIGHTBLUE);
	for(int i = 0; i<b; i++)
	{
		for(int j = 0; j<a; j++)
		{
			TFT_DrawBMP(marioBMP ,blocks[n] ,posX + j*16, posY + i*16);
		}
	}
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void ReadCMDParams(CMD *cmd, uint8_t* data)
{
	uint8_t bit = 1;
	
	cmd->command = 0;
	cmd->parameter = 0;
	
	while(cmd->endIndex >= cmd->startIndex && cmd->command == 0)
	{
		if(isdigit(data[cmd->endIndex]))
		{
			cmd->parameter += bit*(data[cmd->endIndex] - 48);
			bit *= 10;
		}
		else if(isalpha(data[cmd->endIndex]))
		{
			cmd->command = data[cmd->endIndex];
		}
		cmd->endIndex--;
	}
}

void ReadCMDLine(CMD *cmd, uint8_t* Buf)
{
	while(cmd->endIndex >= cmd->startIndex)
	{
		ReadCMDParams(cmd, Buf);
		switch(cmd->command)
		{
			case 's':
			{
				RTC_Time.Seconds = cmd->parameter;
				break;
			}
			case 'm':
			{
				RTC_Time.Minutes = cmd->parameter;
				break;
			}
			case 'h':
			{
				RTC_Time.Hours = cmd->parameter;
				HAL_RTC_SetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
				break;
			}
			case 'd':
			{
				RTC_Date.WeekDay = cmd->parameter;
				HAL_RTC_SetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
				break;
			}
			case 'n':
			{
				RTC_Date.Month = cmd->parameter;
				break;
			}
			case 'l':
			{
				RTC_Date.Date = cmd->parameter;
				break;
			}
			case 'r':
			{
				scrolling.VSPrescaler = cmd->parameter;
				break;
			}
			case 't':
			{
				scrolling.VSStep = cmd->parameter;
				break;
			}
			case 'v':
			{
				scrolling.VSP = cmd->parameter;
				break;
			}
			case 'a':
			{
				DrawBlocks(cmd->parameter, 0, 0, 20, 2);
				break;
			}
			
		}
	}
}

void WAV_TIM7_Init(uint32_t prescaler, uint32_t period)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = prescaler;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = period;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void PlaySound(uint8_t* Buf, uint32_t startAddress, uint32_t endAddress)
{
	for(uint8_t* address = Buf + startAddress; address<Buf+endAddress; address = address+2)
	{
		HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_L, 2048 + *(int16_t*)address/16);
		for(int j = 0; j<310; j++); // delay
	}
}

void PlayWAV()
{	
	if(packetProcessed==0)
	{
		HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_L, 2048 + *(int16_t*)(UserRxBufferFS + WAVStartAddress)/64);
		
		if(wavHeader.numChannels==2)
		{
			HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_L, 2048 + *(int16_t*)(UserRxBufferFS + WAVStartAddress + 2)/64);
		}

		WAVStartAddress = WAVStartAddress + wavHeader.numChannels*2;
		
		if(WAVStartAddress>=WAVEndAddress)
		{
			packetProcessed = 1;
			byteCount += WAVEndAddress;
			if(byteCount >= Size)
			{
				__HAL_TIM_DISABLE(&htim7);
//				HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
//				HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
				byteCount = 0;
			}
			USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		}
	}
	
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
	
    break;

  case CDC_GET_LINE_CODING:     

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	
	if(byteCount == 0)
	{
		Type = *(uint16_t*)Buf;
		switch (Type)
		{
			case 0x4D42:
			{
				TFT_busy = 1;
				bitmapHeader.bfSize = *(uint32_t*)(Buf + 0x02);
				bitmapHeader.bfOffBits = *(uint16_t*)(Buf + 0x0A);
				bitmapHeader.bcWidth = *(uint16_t*)(Buf + 0x12);
				bitmapHeader.bcHeight = *(uint16_t*)(Buf + 0x16);
				
				Size = bitmapHeader.bfSize;
				PosFrame = bitmapHeader.bfOffBits/64 * 64;
				Pos = bitmapHeader.bfOffBits%64;
				break;
			}
			
			case 0x5446:
			{
				Size = *(uint32_t*)(Buf + 0x02);
				break;
			}
			
			case 0x4D43:
			{
				cmd1.startIndex = 2;
				cmd1.endIndex = 5;
				ReadCMDParams(&cmd1, Buf);
				Size = cmd1.parameter;
				cmd1.startIndex = 6;
				cmd1.endIndex = 7;
				ReadCMDParams(&cmd1, Buf); //Read bfOffBits
				cmd1.startIndex = cmd1.parameter;
				cmd1.endIndex = *Len-1;
				ReadCMDLine(&cmd1, Buf);
				break;
			}
			
			case 0x4952:
			{
				uint8_t dataPos = 36;
				wavHeader.chunkSize = *(uint32_t*)(Buf + 4);
				wavHeader.format = *(uint32_t*)(Buf + 8);
				wavHeader.subChunk1Id = *(uint32_t*)(Buf + 12);
				wavHeader.subChunk1Size = *(uint32_t*)(Buf + 16);
				wavHeader.audioFormat = *(uint16_t*)(Buf + 20);
				wavHeader.numChannels = *(uint16_t*)(Buf + 22);
				wavHeader.sampleRate = *(uint32_t*)(Buf + 24);
				wavHeader.byteRate = *(uint32_t*)(Buf + 28);
				wavHeader.blockAlign = *(uint16_t*)(Buf + 32);
				wavHeader.bitsPerSample = *(uint16_t*)(Buf + 34);
				while(*(uint32_t*)(Buf + dataPos) != 0x61746164)
				{
					dataPos++;
				}
				
				wavHeader.subChunk2Id = *(uint32_t*)(Buf + dataPos);
				wavHeader.subChunk2Size = *(uint32_t*)(Buf + dataPos + 4);
				
				WAV_TIM7_Init(72000000/(wavHeader.sampleRate*8), 7);
				
				Size = wavHeader.chunkSize + 8;
				PosFrame = (dataPos+8)/64 * 64;
				Pos = (dataPos+8)%64;
				break;
			}
			
			default:
			{
				Size = 0;
				break;
			}
			
		}
		
	}
	
	switch (Type)
	{
		case 0x4D42:
		{
			if(byteCount>PosFrame)
			{
				TFT_DrawBMPFromBuffer(Buf, 0, *Len, 1);
			}else if(byteCount == PosFrame)
			{
				TFT_DrawBMPFromBuffer(Buf, Pos, *Len, 0);
			}
			
//			memcpy(marioBMP+byteCount, Buf, *Len);
			break;
		}
		
		case 0x5446:
		{
			memcpy(TahomaLCD+byteCount, Buf, *Len);
			break;
		}
		
		case 0x4952:
		{
			if(byteCount>PosFrame)
			{
				WAVStartAddress = 0;
				WAVEndAddress = *Len;
				packetProcessed = 0;
			}else if(byteCount == PosFrame)
			{
				WAVStartAddress = Pos;
				WAVEndAddress = *Len;
				packetProcessed = 0;
				__HAL_TIM_ENABLE(&htim7);
			}
			break;
		}
		
	}
		
	if(Type != 0x4952)
	{
		byteCount += *Len;
	
		if(byteCount >= Size)
		{
			TFT_busy = 0;
			byteCount = 0;
//			TFT_ClearScreen();
//			TFT_DrawBMP(marioBMP, block2, 0, 0);
		}
		
		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	}
	
  return (USBD_OK);
  /* USER CODE END 6 */ 
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */ 
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

