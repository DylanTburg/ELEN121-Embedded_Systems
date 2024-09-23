#include "main.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"

volatile int seccount = 0;
volatile int mincount = 0;
volatile int tenth = 0;
void displays(uint8_t val) 
{
	 BSP_LCD_GLASS_DisplayChar(&val, POINT_OFF, DOUBLEPOINT_OFF , 5);
}
