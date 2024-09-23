/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
extern volatile int seccount;
volatile int tenseccount = 0;
volatile int tenmincount = 0;
extern volatile int mincount;
extern volatile int tenth;
int cond = 0;
int cond2 = 0;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern LCD_HandleTypeDef hlcd;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

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
  while (1)
  {
  }
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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(JOY_CENTER_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	cond2++;
	if(cond2 == 10) //reset case that makes sure cond2 doesn't get too big
	{
		cond2 = 0;
	}
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(JOY_LEFT_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
	uint8_t reset = 48;
	BSP_LCD_GLASS_DisplayChar(&reset, POINT_OFF, DOUBLEPOINT_OFF ,5 );
	BSP_LCD_GLASS_DisplayChar(&reset, POINT_OFF, DOUBLEPOINT_OFF ,4 );
	BSP_LCD_GLASS_DisplayChar(&reset, POINT_OFF, DOUBLEPOINT_ON ,3 );
	BSP_LCD_GLASS_DisplayChar(&reset, POINT_OFF, DOUBLEPOINT_OFF ,2 );
	seccount = 0;
	tenseccount = 0;
	tenmincount = 0;
	mincount = 0;
	tenth = 0;
	
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{

  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
 
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
	if(cond2 % 2 == 0)
	{
	uint8_t ch;
	uint8_t ch2;
		uint8_t ch3;
		uint8_t ch4;	
	tenth++;
		if(tenth > 9)
		{
			seccount++;
			tenth=0;
			if (seccount> 9) 
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
							BSP_LCD_GLASS_DisplayChar(&ch4, POINT_OFF, DOUBLEPOINT_OFF , 2);
						}
						ch3 = mincount + 48;
						BSP_LCD_GLASS_DisplayChar(&ch3, POINT_OFF, DOUBLEPOINT_ON , 3);
					}
					ch2 = tenseccount + 48;
					BSP_LCD_GLASS_DisplayChar(&ch2, POINT_OFF, DOUBLEPOINT_OFF ,4 );
			}
			ch = seccount + 48;
			BSP_LCD_GLASS_DisplayChar(&ch, POINT_OFF, DOUBLEPOINT_OFF ,5 );
			
		}
	
	}
		/* USER CODE END TIM1_UP_TIM16_IRQn 1 */
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
