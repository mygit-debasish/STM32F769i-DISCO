/*
 * custom_LCD.h
 *
 *  Created on: Feb 6, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOMLCD_H_
#define INC_CUSTOMLCD_H_

#include "stm32f769i_discovery_lcd.h"

#define MAX_COLOR_COUNT (26U)

typedef struct LCDBox
{
	uint32_t XPos;
	uint32_t YPos;
	uint32_t XLength;
	uint32_t YHeight;
	uint32_t borColor;
	uint32_t fillColor;
	uint32_t textColor;
	sFONT *pTextFont;
}LCDBox_t;


typedef enum
{
	COLOR_BLUE = 0,
	COLOR_GREEN,
	COLOR_RED,
	COLOR_CYAN,
	COLOR_MAGENTA,
	COLOR_YELLOW,
	COLOR_LIGHTBLUE,
	COLOR_LIGHTGREEN,
	COLOR_LIGHTRED,
	COLOR_LIGHTCYAN,
	COLOR_LIGHTMAGENTA,
	COLOR_LIGHTYELLOW,
	COLOR_DARKBLUE,
	COLOR_DARKGREEN,
	COLOR_DARKRED,
	COLOR_DARKCYAN,
	COLOR_DARKMAGENTA,
	COLOR_DARKYELLOW,
	COLOR_WHITE,
	COLOR_LIGHTGRAY,
	COLOR_GRAY,
	COLOR_DARKGRAY,
	COLOR_BLACK,
	COLOR_BROWN,
	COLOR_ORANGE,
	COLOR_TRANSPARENT
}FontColor_t;

void setLCDBox(uint32_t XPos, uint32_t YPos, uint32_t XLength, uint32_t YHeight,
		uint32_t borColor, uint32_t fillColr, uint32_t testColr,
		sFONT *pTextFont, LCDBox_t *pLcdBix);

void drawCircleMultiColors(uint8_t bColor);


#endif /* INC_CUSTOMLCD_H_ */
