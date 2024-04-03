/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WD_Pin GPIO_PIN_13
#define WD_GPIO_Port GPIOC
#define IC_OK_Pin GPIO_PIN_14
#define IC_OK_GPIO_Port GPIOC
#define MOTO_IV_Pin GPIO_PIN_15
#define MOTO_IV_GPIO_Port GPIOC
#define T_IN3_Pin GPIO_PIN_3
#define T_IN3_GPIO_Port GPIOC
#define FIEP_Pin GPIO_PIN_0
#define FIEP_GPIO_Port GPIOA
#define MTRP_Pin GPIO_PIN_1
#define MTRP_GPIO_Port GPIOA
#define PWRV_Pin GPIO_PIN_2
#define PWRV_GPIO_Port GPIOA
#define O_DA_Pin GPIO_PIN_4
#define O_DA_GPIO_Port GPIOA
#define OA_DA_Pin GPIO_PIN_5
#define OA_DA_GPIO_Port GPIOA
#define TANK_Pin GPIO_PIN_6
#define TANK_GPIO_Port GPIOA
#define SYNOUT_Pin GPIO_PIN_7
#define SYNOUT_GPIO_Port GPIOA
#define OUTV2_Pin GPIO_PIN_4
#define OUTV2_GPIO_Port GPIOC
#define OUTI2_Pin GPIO_PIN_5
#define OUTI2_GPIO_Port GPIOC
#define SMPL_Pin GPIO_PIN_0
#define SMPL_GPIO_Port GPIOB
#define FPWM_Pin GPIO_PIN_1
#define FPWM_GPIO_Port GPIOB
#define MISS_Pin GPIO_PIN_2
#define MISS_GPIO_Port GPIOB
#define SA_Pin GPIO_PIN_12
#define SA_GPIO_Port GPIOB
#define SA_EXTI_IRQn EXTI15_10_IRQn
#define VGEAR_Pin GPIO_PIN_13
#define VGEAR_GPIO_Port GPIOB
#define CC_CV_Pin GPIO_PIN_14
#define CC_CV_GPIO_Port GPIOB
#define A_D_L_Pin GPIO_PIN_15
#define A_D_L_GPIO_Port GPIOB
#define ARCF_Pin GPIO_PIN_7
#define ARCF_GPIO_Port GPIOC
#define HSW_Pin GPIO_PIN_8
#define HSW_GPIO_Port GPIOC
#define BBR_Pin GPIO_PIN_9
#define BBR_GPIO_Port GPIOC
#define PWM_ON_Pin GPIO_PIN_8
#define PWM_ON_GPIO_Port GPIOA
#define ELCPWM_TM1_Pin GPIO_PIN_9
#define ELCPWM_TM1_GPIO_Port GPIOA
#define AOV_SAI_Pin GPIO_PIN_10
#define AOV_SAI_GPIO_Port GPIOA
#define AOC_SAI_Pin GPIO_PIN_11
#define AOC_SAI_GPIO_Port GPIOA
#define MDFY_Pin GPIO_PIN_12
#define MDFY_GPIO_Port GPIOA
#define MIG_MMA_Pin GPIO_PIN_15
#define MIG_MMA_GPIO_Port GPIOA
#define FEED_Pin GPIO_PIN_12
#define FEED_GPIO_Port GPIOC
#define START_Pin GPIO_PIN_2
#define START_GPIO_Port GPIOD
#define TS_Pin GPIO_PIN_3
#define TS_GPIO_Port GPIOB
#define KZBH_Pin GPIO_PIN_4
#define KZBH_GPIO_Port GPIOB
#define ARC_Pin GPIO_PIN_5
#define ARC_GPIO_Port GPIOB
#define ER4_Pin GPIO_PIN_6
#define ER4_GPIO_Port GPIOB
#define ER2_Pin GPIO_PIN_7
#define ER2_GPIO_Port GPIOB
#define ER1_Pin GPIO_PIN_8
#define ER1_GPIO_Port GPIOB
#define ER0_Pin GPIO_PIN_9
#define ER0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct 
{
	uint8_t Run;               //任务状态：Run/Stop
	uint16_t TIMCount;         //定时计数器
	uint16_t TRITime;          //重载计数器
	void (*TaskHook) (void); //任务函数
} TASK_COMPONENTS;  


typedef enum{		//Control MCU state
	GMAW_STANDBY = 0 ,
	
	GMAW_INVERTER_ON ,
	GMAW_RUN_IN ,
	GMAW_ARC_DETECT ,
	GMAW_FLASH_START ,
	GMAW_STABILIZE ,
	
	GMAW_DWELL ,
	GMAW_END_STAGE ,
	GMAW_BURN_BACK ,
	GMAW_WIRE_SHARP ,
	
	GMAW_ERROR ,
} GMAWStatus_t;

 typedef struct {
	uint16_t weld_speed;
	uint16_t weld_volt;
	uint8_t weld_Inductor;
	uint16_t weld_vc;
	uint8_t weld_vslope;

}GAS_WELDING_DATA;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
