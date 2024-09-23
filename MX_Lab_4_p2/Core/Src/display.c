#include "main.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"

volatile int seccount = 0;
volatile int mincount = 0;
volatile int tenth = 0;
char ch = 'X';
void display_test(void) 
{
	 BSP_LCD_GLASS_DisplayChar((uint8_t*)&ch, POINT_OFF, DOUBLEPOINT_OFF ,3 );
}
