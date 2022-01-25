/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_TOUCH_H__
#define __ILI9341_TOUCH_H__

#include <stdbool.h>
#include <main.h>

/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 1.3 Mbit speed, better ~650 Kbit to be save.
#define ILI9341_TOUCH_SPI_PORT hspi2
extern SPI_HandleTypeDef ILI9341_TOUCH_SPI_PORT;

#define ILI9341_TOUCH_IRQ_Pin       IRQ_Pin // Arduino D5
#define ILI9341_TOUCH_IRQ_GPIO_Port IRQ_GPIO_Port
#define ILI9341_TOUCH_CS_Pin        TOUCH_CS_Pin // Arduino D2
#define ILI9341_TOUCH_CS_GPIO_Port  TOUCH_CS_GPIO_Port

// change depending on screen orientation
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define ILI9341_TOUCH_MIN_RAW_X 2000
#define ILI9341_TOUCH_MAX_RAW_X 30100
#define ILI9341_TOUCH_MIN_RAW_Y 2000
#define ILI9341_TOUCH_MAX_RAW_Y 32000

// call before initializing any SPI devices
void ILI9341_TouchUnselect();

bool ILI9341_TouchPressed();
bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y);

#endif // __ILI9341_TOUCH_H__
