//TFT FUNCTIONS BEGIN
#include "stm32f3xx_hal.h"
#include "usbd_cdc_if.h"

typedef struct 
{
	uint16_t TFA;
	uint16_t VSA;
	uint16_t BFA;
	uint16_t VSP;
	uint8_t VSStep;
	uint8_t VSPrescaler;
	uint8_t VSCounter;
}VerticalScrolling;

void TFT_SendStreamData(uint16_t data);

void TFT_SendDataInStream(uint16_t data, uint32_t n);

void TFT_SendCMD(uint16_t cmd);

void TFT_ReadDataStream(uint32_t XY);

//uint16_t TFT_ReadStreamData();

//uint16_t TFT_ReadData();

void TFT_ReadToBuffer(int16_t XL, int16_t XR, int16_t YU, int16_t YD);

void TFT_SendData(uint16_t data);

void TFT_SetCol(uint16_t StartCol,uint16_t EndCol);

void TFT_SetPage(uint16_t StartPage,uint16_t EndPage);

void TFT_SetXY(uint16_t poX, uint16_t poY);

void TFT_SetPixel(uint16_t poX, uint16_t poY,uint16_t color);

void  TFT_DrawHorizontalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color);

void TFT_DrawCharTransparent(char N, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawChar(char N, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawStringNumber(char string[],uint8_t Length,uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawTimeLine(RTC_TimeTypeDef* time,uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_VerticalScrollingStartAddress(uint16_t VSP);

void TFT_IncVerticalScrolling(VerticalScrolling *scrolling);

void TFT_VerticalScrollingDefinition(VerticalScrolling *scrolling);

void TFT_DrawFromBuffer(int16_t XL, int16_t XR, int16_t YU, int16_t YD);

void TFT_DrawBMPFromBuffer(uint8_t* Buf, uint32_t startAddress, uint32_t endAddress, uint8_t continueWrite);

void TFT_DrawBMP(uint8_t* Buf,BITMAP header ,uint16_t posX, uint16_t posY);

void TFT_DrawString(char string[],uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawNumber(uint16_t number, uint8_t Len, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawMonth(RTC_DateTypeDef* date, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawDay(RTC_DateTypeDef* date, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY);

void TFT_DrawLine( uint16_t x0,uint16_t y0,uint16_t x1, uint16_t y1,uint16_t color);

void TFT_DrawVerticalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color);

void TFT_DrawRectangle(uint16_t poX, uint16_t poY, uint16_t length, uint16_t width,uint16_t color);

void TFT_DrawCircle(int poX, int poY, int r,uint16_t color);

void TFT_FillCircle(int poX, int poY, int r,uint16_t color);

void TFT_ClearScreen(void);

uint16_t constrain(int16_t a, int16_t b, uint16_t c);

void TFT_FillScreen(int16_t XL, int16_t XR, int16_t YU, int16_t YD, uint16_t color);

void TFT_Init(uint8_t orient);

//TFT FUNCTIONS END
