/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include "song16.h"
#include "fatfs.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include <ctype.h> // added after changing SFN to LFN
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */




// IR Remote Variables
int irval;
int startCount;
int i = 0;
int j = 0;
int flag = 0;
int k = 0;
int LCD_StartFlag = 0;
int stringIndex = 0;
uint32_t mag;
char glassString[100];
int count = 0;
extern char tempString[300];
extern int globalCount;
extern volatile int seccount;
volatile int tenseccount = 0;
volatile int tenmincount = 0;
extern volatile int mincount;
extern volatile int tenth; 
extern unsigned int irdat[];
extern char songlist[100][300];
extern int current_song_index;
uint32_t x;
int lightFlag = 0;
int IrCount = 0;
int data = 0;
int IrFlag =0;
int volume2 = DFLT_VOLUME;
int timeFlag = 0;
extern int ch0;
int max=0;
int min=0;
uint32_t start = 0x00000000;
uint32_t end = 0xffffffff;
uint32_t red = 0xe40000ff;
uint32_t blue = 0xe4ff0000;
uint32_t lightGreen = 0xe400ff00;
uint32_t green = 0xe4228B22;
uint32_t yellow = 0xe400a8ff;
uint32_t orange = 0xe40024ff;
uint32_t purple = 0xe4cf00ff;
uint32_t spectrum[3] = {0xe400ff00, 0xe400a8ff, 0xe40000ff};
uint32_t black = 0xe0000000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void write_DAC1Ch2(int16_t dacval, int volume);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi32(unsigned int c) 
{ 
	int i;
  for (i = 0; i < 32; ++i) {
      if (c & 0x80000000)
				{
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
				}
        else
				{
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				}
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
        c <<= 1;
		}
}
void send_array(uint32_t header, uint32_t color, uint32_t end)
{
	int i;
	spi32(header);
	for(i=0; i < 60; i++)
	{
		spi32(color);
	}
	spi32(end);
}
void bluemove (uint32_t header, uint32_t color, uint32_t end)
{
	uint32_t blue = 0xe4ff0000;
	int i;
	spi32(header);
	for(i=0; i < 60; i++)
	{
		if (i==globalCount)
		{
			spi32(blue);
		}
		else
		{
			spi32(color);
		}
		
	}
	spi32(end);
	globalCount++;
	if(globalCount > 60)
	{
		globalCount=0; //
	}
	
}
// added after changing SFN to LFN
char *strupr(char s[])
{
  char	*p;

  for (p = s; *p; ++p)
    *p = toupper(*p);

  return(s);

}	
// added after changing SFN to LFN
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern DAC_HandleTypeDef hdac1;
extern LCD_HandleTypeDef hlcd;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
/* USER CODE BEGIN EV */
extern int16_t audiobuffer[NUM_ABUF][ABUF_SIZE]; 	/* File read buffer for audio */
extern volatile bool abuf_full[NUM_ABUF];
extern bool file_open;
extern int fill_buf;
extern int lastbuffer;
extern volatile uint8_t volume;
extern FIL MyFile;



/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
	
	if(flag !=2)
	{
		uint8_t ch1;
		uint8_t ch2;
		uint8_t ch3;
		uint8_t ch4;
		uint8_t one = 49;
        uint8_t zero = 48; 
		uint8_t num3=48;
		uint8_t num4 = 48;
		uint8_t num5 = 48;
        uint8_t nul = 0;
        if(timeFlag == 0 && tenth % 5 ==0 && tenth > 2)  
        {
					BSP_LCD_GLASS_Clear();
					glassString[0] = tempString[stringIndex-1];
                glassString[1] = tempString[stringIndex];
                glassString[2] = tempString[stringIndex+1];
                glassString[3] = tempString[stringIndex+2];
                glassString[4] = tempString[stringIndex+3];
                 glassString[5] = tempString[stringIndex+4];
					//old method. made intro stutter
                /*uint8_t p0 = tempString[stringIndex-1]; 
                uint8_t p1 = tempString[stringIndex];
                uint8_t p2 = tempString[stringIndex+1];
                uint8_t p3 = tempString[stringIndex+2];
                uint8_t p4 = tempString[stringIndex+3];
                uint8_t p5 = tempString[stringIndex+4];
                BSP_LCD_GLASS_DisplayChar(&p0, POINT_OFF, DOUBLEPOINT_OFF , 0);
                BSP_LCD_GLASS_DisplayChar(&p1, POINT_OFF, DOUBLEPOINT_OFF , 1);
                BSP_LCD_GLASS_DisplayChar(&p2, POINT_OFF, DOUBLEPOINT_OFF , 2);
                BSP_LCD_GLASS_DisplayChar(&p3, POINT_OFF, DOUBLEPOINT_OFF , 3);
                BSP_LCD_GLASS_DisplayChar(&p4, POINT_OFF, DOUBLEPOINT_OFF , 4);
                BSP_LCD_GLASS_DisplayChar(&p5, POINT_OFF, DOUBLEPOINT_OFF , 5);*/
								strupr(glassString); // added after changing SFN to LFN
								BSP_LCD_GLASS_DisplayString((uint8_t*)(glassString));
                stringIndex++;
        }
		tenth++;
		if(tenth > 9)
		{
			seccount++;
			tenth=0;
			if(tenseccount > 0 && glassString[0] == NULL && glassString[1] == NULL && glassString[2] == NULL && glassString[3] == NULL && glassString[4] == NULL && glassString[5] == NULL && timeFlag==0)
			{
				timeFlag=1;
                BSP_LCD_GLASS_DisplayChar(&nul, POINT_OFF, DOUBLEPOINT_OFF , 0);
                BSP_LCD_GLASS_DisplayChar(&nul, POINT_OFF, DOUBLEPOINT_OFF , 1);
               // BSP_LCD_GLASS_DisplayChar(&zero, POINT_OFF, DOUBLEPOINT_OFF , 2);
               // BSP_LCD_GLASS_DisplayChar(&zero, POINT_OFF, DOUBLEPOINT_ON , 3);
								num3 += tenseccount;
								num4+=mincount;
								num5 +=tenmincount;
                BSP_LCD_GLASS_DisplayChar(&num3, POINT_OFF, DOUBLEPOINT_OFF , 4);
								BSP_LCD_GLASS_DisplayChar(&num4, POINT_OFF, DOUBLEPOINT_ON , 3);
								BSP_LCD_GLASS_DisplayChar(&num5, POINT_OFF, DOUBLEPOINT_OFF , 2);
			}
			if (seccount > 9) 
			{
				seccount = 0;
				tenseccount++;
				if(tenseccount > 5)
					{
						tenseccount = 0;
						mincount++;
						if (mincount > 9)
						{
							mincount = 0;
							tenmincount++;
							if (tenmincount > 5)
							{
								mincount = 0;
								tenmincount = 0;
							}
							ch4 = tenmincount + 48;
							if(timeFlag==1)
							{
								BSP_LCD_GLASS_DisplayChar(&ch4, POINT_OFF, DOUBLEPOINT_OFF , 2);
							}
						}
						ch3 = mincount + 48;
						if(timeFlag==1)
							{
								BSP_LCD_GLASS_DisplayChar(&ch3, POINT_OFF, DOUBLEPOINT_ON , 3);
							}
					}
					ch2 = tenseccount + 48;
					if(timeFlag==1)
							{
								BSP_LCD_GLASS_DisplayChar(&ch2, POINT_OFF, DOUBLEPOINT_OFF , 4);
							}
			}
			ch1 = seccount + 48;
			if(timeFlag==1)
							{
								BSP_LCD_GLASS_DisplayChar(&ch1, POINT_OFF, DOUBLEPOINT_OFF , 5);
							}
			BSP_LCD_GLASS_BarLevelConfig(ch0);
		}
		LCD_StartFlag = 1;
	}
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */
	if(lightFlag ==1)
	{
		int buf;
		if(flag == 0)
		{
			buf=0;
		}
		if(flag == 1)
		{
			buf=1;
		}	
		mag = (max-min)/1092;
		max=0;
		min=0;
		send_array(start,black,end);				
		spi32(start);
					for(int x=0; x < mag; x++)
						{
							if(x < 20)
							{
								spi32(green);
							}
								
							if(x >=20 && x < 39)
							{
								spi32(yellow);
							}
								
							if (x >=40)
							{
								spi32(red);
							} 
						}
						lightFlag=0; 
	}
  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	
  /***************************************************/
	/* Put your code here to play a song							 */
  if(lastbuffer == 0)
	{
		lightFlag = 0;
		if (flag == 0)
		{
				if(abuf_full[0] == true)
				{
					// Switch to the next buffer
					 write_DAC1Ch2(audiobuffer[k][i], volume2);
					 i=(i+1)%ABUF_SIZE;
				}
				if(audiobuffer[k][i] > max)
				{
					max = audiobuffer[k][i];
				}
				if(audiobuffer[k][i] < min)
				{
					min = audiobuffer[k][i];
				}
				if(i == 0)
				{
						abuf_full[0] = false;
						flag = 1;
						k++;
				}
		}
		if (flag == 1)
		{
				if(abuf_full[1] == true)
				{
					// Switch to the next buffer
					 write_DAC1Ch2(audiobuffer[k][j], volume2);
					 j=(j+1)%ABUF_SIZE;
				}
				if(audiobuffer[k][j] > max)
				{
					max = audiobuffer[k][j];
				}
				if(audiobuffer[k][j] < min)
				{
					min = audiobuffer[k][j];
				}
				if(j == 0)
				{
						abuf_full[1] = false;
						flag = 0;
						k--;
				}
		}
		lightFlag = 1;
	}
	else if(flag !=2)
	{
		write_DAC1Ch2((uint16_t) audiobuffer[k][count], volume2);
		count = (count+1)%ABUF_SIZE;
		if((count*2) > (lastbuffer-1))
		{
			flag = 2;
		}
		if(audiobuffer[k][count] > max)
		{
			max = audiobuffer[k][count];
		}
		if(audiobuffer[k][count] < min)
		{
			min = audiobuffer[k][count];
		}
	} 
  /***************************************************/

			
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
	x = HAL_GPIO_ReadPin(IR_IN_GPIO_Port, IR_IN_Pin);
	if(IrFlag == 0)
	{		
		if(x == 0 || data == 1)
		{
			data = 1;
			if(x==0)
			{
				irdat[IrCount] = 1;
				IrCount = (IrCount +1) %SAMPLE_COUNT;
			}
			else
			{
				irdat[IrCount]=0;
				IrCount= (IrCount +1) %SAMPLE_COUNT;
			}
			if(IrCount == SAMPLE_COUNT-1)
			{
				IrFlag =1;
				data =0;
			}
		}
	}
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles LCD global interrupt.
  */
void LCD_IRQHandler(void)
{
  /* USER CODE BEGIN LCD_IRQn 0 */

  /* USER CODE END LCD_IRQn 0 */
  /* USER CODE BEGIN LCD_IRQn 1 */

  /* USER CODE END LCD_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
