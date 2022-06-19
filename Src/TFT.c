//TFT FUNCTIONS BEGIN
#include "stm32f3xx_hal.h"
#include "TFT.h"

#define TFT_StartWriteStream() GPIOC->BSRR = GPIO_PIN_2|GPIO_PIN_0 //set DC/RS and RD high
#define TFT_EndWriteStream() GPIOC->BRR = GPIO_PIN_2 //set DC/RS low

#define TFT_StartReadStream() GPIOC->BSRR = GPIO_PIN_2|GPIO_PIN_1 // set DC/RS and WR high
#define TFT_EndReadStream() GPIOC->BRR = GPIO_PIN_2; //set DC/RS low

uint16_t MAX_X = 0;
uint16_t MAX_Y = 0;
uint16_t MIN_X = 0; 
uint16_t MIN_Y = 0;

uint16_t RED	= 0xf800;
uint16_t GREEN = 0x07e0;
uint16_t BLUE	= 0x001f;
uint16_t BLACK =	0x0000;
uint16_t YELLOW	= 0xffe0;
uint16_t WHITE	= 0xfffe;
uint16_t LIGHTBLUE	= 0x5CBF;

uint16_t CYAN	= 0x07ff;
uint16_t BRIGHT_RED	= 0xf810;
uint16_t GRAY1 = 0x8410;
uint16_t GRAY2 = 0x4208;

uint16_t clearTimeWidth;
uint16_t clearTimeHeight;

uint8_t TFT_busy = 1;

char TFT_Monday[] = "MONDAY";
char TFT_Tuesday[] = "TUESDAY";
char TFT_Wednesday[] = "WEDNESDAY";
char TFT_Thursday[] = "THURSDAY";
char TFT_Friday[] = "FRIDAY";
char TFT_Saturday[] = "SATURDAY";
char TFT_Sunday[] = "SUNDAY";

char* TFT_Weekdays[] = {TFT_Monday, TFT_Tuesday, TFT_Wednesday, TFT_Thursday, TFT_Friday, TFT_Saturday, TFT_Sunday};

char TFT_January[] = "JANUARY";
char TFT_February[] = "FEBRUARY";
char TFT_March[] = "MARCH";
char TFT_April[] = "APRIL";
char TFT_May[] = "MAY";
char TFT_June[] = "JUNE";
char TFT_July[] = "JULY";
char TFT_August[] = "AUGUST";
char TFT_September[] = "SEPTEMBER";
char TFT_October[] = "OCTOBER";
char TFT_November[] = "NOVEMBER";
char TFT_December[] = "DECEMBER";

char* TFT_Months[] = {TFT_January, TFT_February, TFT_March, TFT_April, TFT_May, TFT_June, TFT_July, TFT_August, TFT_September, TFT_October, TFT_November, TFT_December};


//uint16_t ReadBuffer[300];

uint8_t TahomaLCD[12000];

extern int newData;

extern BITMAP* fontHeader;
extern BITMAP bitmapHeader;

void TFT_SendData(uint16_t data)
{
	GPIOC->BRR = GPIO_PIN_1; //set WR low
	GPIOD->ODR = data; //send data via PortD
	GPIOC->BSRR = GPIO_PIN_1; //set WR high
}

void TFT_SendDataStream(uint16_t data, uint32_t XY)
{	
	TFT_SendCMD(0x2C);
	GPIOD->ODR = data; //send data via PortD
	for(uint32_t i = 0; i < XY; i++)
	{
		GPIOC->BRR = GPIO_PIN_1; //set WR low
		GPIOC->BSRR = GPIO_PIN_1; //set WR high
	}
	
}

void TFT_SendCMD(uint16_t cmd)
{
	GPIOC->BRR = GPIO_PIN_2; // set DC/RS low
	TFT_SendData(cmd);
	GPIOC->BSRR = GPIO_PIN_2; //set DC/RS high
}

void TFT_SetCol(uint16_t StartCol,uint16_t EndCol)
{
	TFT_SendCMD(0x2A);
	TFT_SendData(StartCol>>8);
	TFT_SendData(StartCol);
	TFT_SendData(EndCol>>8);
	TFT_SendData(EndCol);
}

void TFT_SetPage(uint16_t StartPage,uint16_t EndPage)
{
	TFT_SendCMD(0x2B);
	TFT_SendData(StartPage>>8);
	TFT_SendData(StartPage);
	TFT_SendData(EndPage>>8);
	TFT_SendData(EndPage);
}

//void TFT_ReadDataStream(uint32_t XY)
//{
//	//GPIOC->BSRR = GPIO_PIN_2|GPIO_PIN_1; // set DC/RS and WR high
//	//GPIOC->BSRR = GPIO_PIN_0; //set RD high
//	
//	TFT_SendCMD(0x002E);
//	
//	GPIOC->BRR = GPIO_PIN_1; //set RD low
//	__NOP;
//	GPIOC->BSRR = GPIO_PIN_1; //set RD high
//	__NOP;
//	
//	TFT_SendCMD(0x002E);
//	GPIOC->BRR = GPIO_PIN_1; //set RD low
//	__NOP;
//	GPIOC->BSRR = GPIO_PIN_1; //set RD high
//	__NOP;
//	
//	for(uint16_t i = 0; i < 200; i++)
//	{
//		TFT_SendCMD(0x002E);
//		GPIOC->BRR = GPIO_PIN_1; //set RD low
//		//for (int i = 0; i<200; i++) __NOP;
//		__NOP;
//		ReadBuffer[i] = GPIOD->IDR;
//		__NOP;
//		GPIOC->BSRR = GPIO_PIN_1; //set RD high
//		__NOP;
//	}
//	

//}

//uint16_t TFT_ReadData()
//{
//	uint16_t data;
//	
//	GPIOC->BRR = GPIO_PIN_0; //set RD low
//	
//	data = GPIOD->IDR;
//	
//	GPIOC->BSRR = GPIO_PIN_0; //set RD high
//	
//	return data;
//}

//void TFT_ReadToBuffer(int16_t XL, int16_t XR, int16_t YU, int16_t YD)
//{
//	uint32_t XY;
//	if(XL > XR)
//	{
//		XL = XL^XR;
//		XR = XL^XR;
//		XL = XL^XR;
//	}
//	if(YU > YD)
//	{
//		YU = YU^YD;
//		YD = YU^YD;
//		YU = YU^YD;
//	}
//	XL = constrain(XL, MIN_X,MAX_X);
//	XR = constrain(XR, MIN_X,MAX_X);
//	YU = constrain(YU, MIN_Y,MAX_Y);
//	YD = constrain(YD, MIN_Y,MAX_Y);

//	XY = (XR-XL+1);
//	XY = XY*(YD-YU+1);
//	
//	TFT_SetCol(0,19);
//	TFT_SetPage(0,19);
//	
//	TFT_ReadDataStream(400);
//	
//}

void TFT_SetOrientation(uint8_t orient)
{
	TFT_SendCMD(0x36);
	switch (orient)
	{
		case 0: TFT_SendData(0x48);
				break;
		case 1: TFT_SendData(0xA8);
				break;
		case 2: TFT_SendData(0x88);
				break;
		case 3: TFT_SendData(0xE8);
				break;
	}
	if (orient == 0 || orient == 2)
	{
		MAX_X = 239;
		MAX_Y = 319;
	}
	else
	{
		MAX_X = 319;
		MAX_Y = 239;
	}
}

void TFT_SetXY(uint16_t poX, uint16_t poY)
{
	TFT_SetCol(poX, poX);
	TFT_SetPage(poY, poY);
}

void TFT_SetPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
	TFT_SetXY(poX, poY);
	TFT_SendCMD(0x2C);
	TFT_SendData(~color);
}

void  TFT_DrawHorizontalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color)
{
	TFT_SetCol(poX,poX + length);
	TFT_SetPage(poY,poY);
	TFT_SendDataStream(~color, length);
}

void TFT_DrawVerticalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color)
{
	TFT_SetCol(poX,poX);
	TFT_SetPage(poY,poY+length);
	TFT_SendDataStream(~color, length);
}

void TFT_DrawLine( uint16_t x0,uint16_t y0,uint16_t x1, uint16_t y1,uint16_t color)
{
	int x = x1-x0;
	int y = y1-y0;
	int dx = abs(x), sx = x0<x1 ? 1 : -1;
	int dy = -abs(y), sy = y0<y1 ? 1 : -1;
	int err = dx+dy, e2;                                                /* error value e_xy             */
	for (;;){                                                           /* loop                         */
		TFT_SetPixel(x0,y0,color);
		e2 = 2*err;
		if (e2 >= dy) {                                                 /* e_xy+e_x > 0                 */
			if (x0 == x1) break;
			err += dy; x0 += sx;
		}
		if (e2 <= dx) {                                                 /* e_xy+e_y < 0                 */
			if (y0 == y1) break;
			err += dx; y0 += sy;
		}
	}

}

void TFT_DrawRectangle(uint16_t poX, uint16_t poY, uint16_t length, uint16_t width,uint16_t color)
{
	TFT_DrawHorizontalLine(poX, poY, length, color);
	TFT_DrawHorizontalLine(poX, poY+width, length, color);
	TFT_DrawVerticalLine(poX, poY, width,color);
	TFT_DrawVerticalLine(poX + length, poY, width,color);
}

void TFT_DrawCircle(int poX, int poY, int r,uint16_t color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do {
		TFT_SetPixel(poX-x, poY+y,color);
		TFT_SetPixel(poX+x, poY+y,color);
		TFT_SetPixel(poX+x, poY-y,color);
		TFT_SetPixel(poX-x, poY-y,color);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);
}

void TFT_FillCircle(int poX, int poY, int r,uint16_t color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do {

		TFT_DrawVerticalLine(poX-x, poY-y, 2*y, color);
		TFT_DrawVerticalLine(poX+x, poY-y, 2*y, color);

		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);

}

void TFT_ClearScreen(void)
{
	TFT_SetCol(0, MAX_X);
	TFT_SetPage(0, MAX_Y);                                                 /* start to Send to display ram */
	TFT_SendDataStream(~0x0000, 76800);
}

uint16_t constrain(int16_t a, int16_t b, uint16_t c)
{
	if (a < b)
	{
		return b;
	}
	if (c < a)
	{
		return c;
	}
	else return a;
}

void TFT_FillScreen(int16_t XL, int16_t XR, int16_t YU, int16_t YD, uint16_t color)
{
	uint32_t XY;
	if(XL > XR)
	{
		XL = XL^XR;
		XR = XL^XR;
		XL = XL^XR;
	}
	if(YU > YD)
	{
		YU = YU^YD;
		YD = YU^YD;
		YU = YU^YD;
	}
	XL = constrain(XL, MIN_X,MAX_X);
	XR = constrain(XR, MIN_X,MAX_X);
	YU = constrain(YU, MIN_Y,MAX_Y);
	YD = constrain(YD, MIN_Y,MAX_Y);

	XY = (XR-XL+1);
	XY = XY*(YD-YU+1);

	TFT_SetCol(XL,XR);
	TFT_SetPage(YU,YD);
	
	TFT_SendDataStream(~color, XY);
}

void TFT_DrawChar(char N, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	uint16_t charSize = (fontHeader->bcWidth/8)*fontHeader->bcHeight;
	uint16_t charIndex = fontHeader->bfOffBits + charSize*(N-32);
	
	TFT_SetCol(posX,posX+fontHeader->bcWidth-1);
	TFT_SetPage(posY,posY+fontHeader->bcHeight-1);
	TFT_SendCMD(0x2C);  
	
	for(uint16_t line = charIndex; line<charIndex+charSize; line++)
	{
		for(uint8_t mask = 0x80; mask!=0; mask>>=1)
		{
			if(TahomaLCD[line] & mask)
			{
				TFT_SendData(~color1);
			}else{
				TFT_SendData(~color2);
			}
		}
	}

}

void TFT_DrawString(char string[], uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	uint16_t i = 0;
	
	while(string[i]!=0)
	{
		TFT_DrawChar(string[i], color1, color2, posX + fontHeader->bcWidth*i, posY);
		i++;
	}
}

void TFT_DrawStringNumber(char string[],uint8_t Len,uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	int i = 0;
	int j = 0;
	for(int n = Len-1; n>=0; n--)
	{
		if(string[n]==0){
			TFT_DrawChar('0', color1, color2, posX + fontHeader->bcWidth*i, posY);
			i++;
		}else{
			TFT_DrawChar(string[n], color1, color2, posX + fontHeader->bcWidth*(Len - j - 1), posY);
			j++;
		}
	}
}

void TFT_DrawNumber(uint16_t number, uint8_t Len, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	char String[Len+1];
	sprintf(String,"%d" ,number);
	TFT_DrawStringNumber(String, Len, color1, color2, posX, posY);
}

void TFT_DrawMonth(RTC_DateTypeDef* date, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	TFT_FillScreen(posX, posX+9*clearTimeWidth, posY+clearTimeHeight-1, posY, LIGHTBLUE);
	TFT_DrawString(TFT_Months[date->Month - 1], color1, color2, posX, posY);
}

void TFT_DrawDay(RTC_DateTypeDef* date, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	TFT_FillScreen(posX, posX+9*clearTimeWidth, posY+clearTimeHeight-1, posY, LIGHTBLUE);
	TFT_DrawString(TFT_Weekdays[date->WeekDay - 1], color1, color2, posX, posY);
}

void TFT_DrawTimeLine(RTC_TimeTypeDef* time, uint16_t color1, uint16_t color2, uint16_t posX, uint16_t posY)
{
	char Hours[3];
	char Minutes[3];
	char Seconds[3];
//	char mSeconds[5];
	
	snprintf(Hours, 3, "%u", time->Hours);
	snprintf(Minutes, 3, "%u", time->Minutes);
	snprintf(Seconds, 3, "%u", time->Seconds);
//		snprintf(mSeconds, 5, "%u", time->SubSeconds);
	
	//TFT_FillScreen(10, posX+12*clearTimeWidth, posY+clearTimeHeight-1, 120, LIGHTBLUE);
	TFT_DrawStringNumber(Hours, 2, color1, color2, posX, posY);
	TFT_DrawChar(':', WHITE, LIGHTBLUE, posX+2*fontHeader->bcWidth, posY);
	TFT_DrawStringNumber(Minutes, 2,color1, color2, posX+3*fontHeader->bcWidth, posY);
	TFT_DrawChar(':', WHITE, LIGHTBLUE, posX+5*fontHeader->bcWidth, posY);
	TFT_DrawStringNumber(Seconds, 2, color1, color2, posX+6*fontHeader->bcWidth, posY);
//		TFT_DrawChar(':', WHITE, LIGHTBLUE, posX+8*fontHeader.bcWidth, posY);
//		TFT_DrawTime(mSeconds, 4, WHITE, LIGHTBLUE, posX+9*fontHeader.bcWidth, posY);
}

void TFT_VerticalScrollingStartAddress(uint16_t VSP)
{
	TFT_SendCMD(0x37);

	TFT_SendData(VSP>>8);
	TFT_SendData(VSP);
}

void TFT_IncVerticalScrolling(VerticalScrolling *scrolling)
{
	TFT_VerticalScrollingStartAddress(scrolling->VSP);
	
	if(scrolling->VSCounter < scrolling->VSPrescaler)
	{
		scrolling->VSCounter++;
	}else{
		if(scrolling->VSP>1)
		{
			scrolling->VSP = scrolling->VSP - scrolling->VSStep;
		}else{
			scrolling->VSP = 320;
		}
		scrolling->VSCounter = 1;
	}
}

void TFT_VerticalScrollingDefinition(VerticalScrolling *scrolling)
{
	TFT_SendCMD(0x33);
	TFT_SendData(scrolling->TFA>>8);
	TFT_SendData(scrolling->TFA);
	TFT_SendData(scrolling->VSA>>8);
	TFT_SendData(scrolling->VSA);
	TFT_SendData(scrolling->BFA>>8);
	TFT_SendData(scrolling->BFA);
}

//void TFT_DrawFromBuffer(int16_t XL, int16_t XR, int16_t YU, int16_t YD)
//{
//	uint32_t XY;
//	if(XL > XR)
//	{
//		XL = XL^XR;
//		XR = XL^XR;
//		XL = XL^XR;
//	}
//	if(YU > YD)
//	{
//		YU = YU^YD;
//		YD = YU^YD;
//		YU = YU^YD;
//	}
//	XL = constrain(XL, MIN_X,MAX_X);
//	XR = constrain(XR, MIN_X,MAX_X);
//	YU = constrain(YU, MIN_Y,MAX_Y);
//	YD = constrain(YD, MIN_Y,MAX_Y);

//	XY = (XR-XL+1);
//	XY = XY*(YD-YU+1);

//	TFT_SetCol(0,19);
//	TFT_SetPage(220,239);
//	
//	for(uint16_t* address = ReadBuffer; address<ReadBuffer+300; address++)
//	{
//		TFT_SendData(*address);
//	}
//
//}

void TFT_DrawBMPFromBuffer(uint8_t* Buf, uint32_t startAddress, uint32_t endAddress, uint8_t continueWrite)
{
	if(continueWrite == 1)
	{
		TFT_SendCMD(0x3C); 
	}else{
		TFT_SetCol(0,bitmapHeader.bcWidth-1);
		TFT_SetPage(0,bitmapHeader.bcHeight-1);
		TFT_SendCMD(0x2C);
	}

	for(uint8_t* address = Buf + startAddress; address<Buf+endAddress; address = address+2)
	{
		TFT_SendData(~*(uint16_t*)address);
	}
}

void TFT_DrawBMP(uint8_t* Buf,BITMAP header ,uint16_t posX, uint16_t posY)
{
	TFT_SetCol(posX,posX+header.bcWidth-1);
	TFT_SetPage(posY,posY+header.bcHeight-1);
	TFT_SendCMD(0x2C); 
	
	for(uint8_t* address = Buf + header.bfOffBits; address<Buf+header.bfSize; address = address+2)
	{
		TFT_SendData(~*(uint16_t*)address);
	}
}

void TFT_Init(uint8_t orient)
{
	TFT_SendCMD(0x01);
	HAL_Delay(1000);
	//************* Start Initial Sequence **********//
	//Power control A
	TFT_SendCMD(0xCB);
	TFT_SendData(0x39);
	TFT_SendData(0x2C);
	TFT_SendData(0x00);
	TFT_SendData(0x34);
	TFT_SendData(0x02);


	//Power control A
	TFT_SendCMD(0xCF);
	TFT_SendData(0x00);
	TFT_SendData(0xC1);
	TFT_SendData(0x30);


	//Driver timing control A
	TFT_SendCMD(0xE8);
	TFT_SendData(0x85);
	TFT_SendData(0x00);
	TFT_SendData(0x78);


	//Driver timing control B
	TFT_SendCMD(0xEA);
	TFT_SendData(0x00);
	TFT_SendData(0x00);


	//Power on sequence control
	TFT_SendCMD(0xED);
	TFT_SendData(0x64);
	TFT_SendData(0x03);
	TFT_SendData(0x12);
	TFT_SendData(0x81);


	//Pump ratio control
	TFT_SendCMD(0xF7);
	TFT_SendData(0x20);


	//Power Control 1
	TFT_SendCMD(0xC0);
	TFT_SendData(0x23);


	//Power Control 2
	TFT_SendCMD(0xC1);
	TFT_SendData(0x10);


	//VCOM Control 1
	TFT_SendCMD(0xC5);
	TFT_SendData(0x2B);
	TFT_SendData(0x2B);


	//Memory Access Control
	TFT_SetOrientation(orient);

	//Frame Rate Control (In Normal Mode/Full Colors)
	TFT_SendCMD(0xB1);
	TFT_SendData(0x00);
	TFT_SendData(0x10);

	/*
	//Blink Porch Control
	TFT_SendCMD(0xB5);
	TFT_SendData(0x02);
	TFT_SendData(0x02);
	TFT_SendData(0x02);
	TFT_SendData(0x02);
*/

	//Display Function Control
	TFT_SendCMD(0xB6);
	TFT_SendData(0x0A);
	TFT_SendData(0x02);


	//Enable 3G
	TFT_SendCMD(0xF2);
	TFT_SendData(0x02);


	//COLMOD: Pixel Format Set
	TFT_SendCMD(0x3A);
	TFT_SendData(0x05);


	//Gamma Set
	TFT_SendCMD(0x26);
	TFT_SendData(0x01);


	//Positive Gamma Correction
	TFT_SendCMD(0xE0);
	TFT_SendData(0x0F);
	TFT_SendData(0x31);
	TFT_SendData(0x2B);
	TFT_SendData(0x0C);
	TFT_SendData(0x0E);
	TFT_SendData(0x08);
	TFT_SendData(0x4E);
	TFT_SendData(0xF1);
	TFT_SendData(0x37);
	TFT_SendData(0x07);
	TFT_SendData(0x10);
	TFT_SendData(0x03);
	TFT_SendData(0x0E);
	TFT_SendData(0x09);
	TFT_SendData(0x00);


	//Negative Gamma Correction
	TFT_SendCMD(0xE1);
	TFT_SendData(0x00);
	TFT_SendData(0x0E);
	TFT_SendData(0x14);
	TFT_SendData(0x03);
	TFT_SendData(0x11);
	TFT_SendData(0x07);
	TFT_SendData(0x31);
	TFT_SendData(0xC1);
	TFT_SendData(0x48);
	TFT_SendData(0x08);
	TFT_SendData(0x0F);
	TFT_SendData(0x0C);
	TFT_SendData(0x31);
	TFT_SendData(0x36);
	TFT_SendData(0x0F);

	// Sleep Out
	TFT_SendCMD(0x11);

	HAL_Delay(120);

	//Display On
	TFT_SendCMD(0x29);
	
	fontHeader = (BITMAP*)(TahomaLCD+2);
}

//TFT FUNCTIONS END
