/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_adc.h"

//#include "weld_struct.h"

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
uint16_t TM3_temp=0;
uint16_t TM3_count=0;

uint16_t FPWM =0;
uint16_t FPWM_Cai=0;
volatile uint16_t FPWM_Set=0;
int16_t FPWM_Dif=0;
uint16_t FPWM_Prv=0;
uint16_t FPWM_Set_Holder=0;
uint16_t FPWM_Set_count=0;
uint8_t FPWM_Freq_Signal=1;
uint8_t FPWM_Freq_Div=1;
uint16_t FPWM_TEMP_Num=0;
uint16_t FB_FIEP_PWM_Ave=0;


//uint16_t FB_FIEP_I_Ave=0;

uint16_t TM7_count=0;

uint8_t DAC_Set_1=0;

uint8_t TM3_Gap=35;
uint8_t SMPL=4;
uint8_t TM3_max=120;

#define Vol_min 0.2f			//0.2     0.05
#define Vol_max 3.1f			//0.9			1.7
#define FB_max 3.1f				//	2.2
#define FB_min 0.3f					//  0.5

#define FPWM_max 160
#define FPWM_P 0.4f
#define FPWM_I 0.7f

#define FPWM_Div_min 10 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */
//extern ADC_HandleTypeDef hadc2;
extern uint16_t FB_FIEP_NUM;
extern uint16_t FB_OUTV_NUM;
extern uint16_t ST_FPWM_NUM;
extern uint16_t ST_OTDA_NUM;
extern uint16_t ST_EPWM_NUM;
extern DAC_HandleTypeDef hdac;
extern uint16_t FB_FIEP_I;
extern int16_t FB_FIEP_D;
extern uint8_t FEED_SLOW_Signal;
extern uint8_t FEED_Growth_Signal;

extern uint8_t res_count;

extern uint8_t Tasks_Max;
extern TASK_COMPONENTS Task_Comps[4];

extern GAS_WELDING_DATA Gas_a;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(SA_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	uint8_t FPWM_ST_Array=0;	
	
	
	if(TM3_count==0){
		
		FPWM_Cai=FB_FIEP_NUM;

		if(FEED_Growth_Signal==0){	
			if(FEED_SLOW_Signal==0)
			FPWM_Set=Gas_a.weld_speed;
		}
		else{		//Growth FEED SPEED
			FPWM_Set_Holder=Gas_a.weld_speed;
			
			FPWM_Set_count++;
			
			if(FPWM_Set_Holder<FPWM_Set){
				FEED_Growth_Signal=0;
			}
			else
				FPWM_Set+=3*((FPWM_Set_count%2)/1);
		}
		
		FPWM_Dif=FB_FIEP_D;
		
		FB_FIEP_PWM_Ave=FB_FIEP_I;	
		//Get PI&FB SAMPLE		
		
		if(FEED_SLOW_Signal==1&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==RESET){									//Slowly FEED
			FPWM_Set=400;
		}
				
		if(FPWM_Set >= FPWM_Cai  &&  FPWM_Set >= FB_FIEP_PWM_Ave)															//PID FPWM Controll in P&I
			FPWM=FPWM+(uint16_t)((FPWM_Set-FPWM_Cai)*FPWM_P)+(uint16_t)((FPWM_Set-FB_FIEP_PWM_Ave)*FPWM_I);		
		else if(FPWM_Set<FPWM_Cai&&FPWM_Set>=FB_FIEP_PWM_Ave){
			if(FPWM >= (uint16_t)((FPWM_Cai-FPWM_Set)*FPWM_P))
				FPWM=FPWM-(uint16_t)((FPWM_Cai-FPWM_Set)*FPWM_P)+(uint16_t)((FPWM_Set-FB_FIEP_PWM_Ave)*FPWM_I);
			else
				FPWM=0+(uint16_t)((FPWM_Set-FB_FIEP_PWM_Ave)*FPWM_I);
		}
		else if(FPWM_Set>=FPWM_Cai&&FPWM_Set<FB_FIEP_PWM_Ave){
			if(FPWM >= (uint16_t)((FB_FIEP_PWM_Ave-FPWM_Set)*FPWM_I))
				FPWM=FPWM+(uint16_t)((FPWM_Set-FPWM_Cai)*FPWM_P)-(uint16_t)((FB_FIEP_PWM_Ave-FPWM_Set)*FPWM_I);
			else
				FPWM=0+(uint16_t)((FPWM_Set-FPWM_Cai)*FPWM_P);
		}
		else if(FPWM_Set<FPWM_Cai&&FPWM_Set<FB_FIEP_PWM_Ave){
			if(FPWM >= (uint16_t)((FPWM_Cai-FPWM_Set)*FPWM_P)+(uint16_t)((FB_FIEP_PWM_Ave-FPWM_Set)*FPWM_I))
				FPWM=FPWM-(uint16_t)((FPWM_Cai-FPWM_Set)*FPWM_P)-(uint16_t)((FB_FIEP_PWM_Ave-FPWM_Set)*FPWM_I);
			else
				FPWM=0;
		}
		
//		if(FPWM_Dif>0)
//			FPWM=FPWM+(uint16_t)(FPWM_Dif*4);
//		else
//			FPWM=FPWM-(uint16_t)(-4*FPWM_Dif);
		
		if(FPWM>FPWM_Prv){
			if(FPWM-FPWM_Prv>20)
				FPWM=FPWM_Prv+20;
		}
		else{
			if(FPWM_Prv-FPWM>40)
				FPWM=FPWM_Prv-40;
		}
		
		
				
		if(FPWM>FPWM_max)	FPWM=FPWM_max;			//x+y+z=140 4x+2y+z=280 x=35 y=35 z=70
		FPWM_TEMP_Num=FPWM;
		
		FPWM_Prv=FPWM;
		
		if( FPWM<=80 ) {
			FPWM_Freq_Div=4;
			TM3_Gap=30;
			SMPL=9;
		}
		else if(FPWM>80&&FPWM<=120)
		{
			FPWM_Freq_Div=2;
			FPWM=FPWM-40;
			TM3_Gap=30;
			SMPL=9;
		}
		else
		{
			FPWM=FPWM-80;
			FPWM_Freq_Div=1;
			TM3_Gap=30;
			SMPL=9;
		}
						
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		
		
	}
	
	TM3_count++;
	
	if(FPWM>0){
		if(FPWM_Freq_Signal==FPWM_Freq_Div) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);			//FPWM Pulse
		FPWM--;							//FPWM Period
		TM3_temp=TM3_count;
	}
	else if(FPWM==0){
		if(TM3_count<=TM3_temp+TM3_Gap){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else if(TM3_count>TM3_temp+TM3_Gap&&TM3_count<=TM3_temp+TM3_Gap+SMPL){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);			//SMPL Pulse
		}
		else if(TM3_count>TM3_temp+TM3_Gap+SMPL&&TM3_count<=TM3_max){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		else
		{
			TM3_count=0;
			TM3_temp=0;
			
			if(FPWM_Freq_Div==1)	
				FPWM_Freq_Signal=1;																				//Function: To controll Div_num period which generated FPWM Pulse
			else
				FPWM_Freq_Signal=FPWM_Freq_Signal%FPWM_Freq_Div+1;	
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			
		}
	}
	
	
	if(res_count!=0){
		if(res_count==25){        //25-¡·15-¡·25--3.25
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			//BBR Reset
			res_count=0;
		}
		res_count++;
	}
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
	uint8_t i;
	for(i=0; i<Tasks_Max; i++)
	{
		if(Task_Comps[i].TIMCount)    /* If the time is not 0 */
		{
			Task_Comps[i].TIMCount--;  /* Time counter decrement */
			if(Task_Comps[i].TIMCount == 0)  /* If time arrives */
			{
				/*Resume the timer value and try again */
				Task_Comps[i].TIMCount = Task_Comps[i].TRITime;  
				Task_Comps[i].Run = 1;    /* The task can be run */
			}
		}
	}
		
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
