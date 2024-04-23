/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "weld_struct.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TS_On_state 1		//1 start
#define KZBH_On_state 1		//
#define ADI_On_state 1		//1 模拟  0 数字
#define MIGMMA_On_state 0			//1 手工焊   0 气保焊
#define MOTOIV_On_state 1
#define CCCV_On_state 0			//CC/CV 	1 OUT_I 0 OUT_V

#define DAC1_Value 2047

#if TS_On_state
	#define TS_state GPIO_PIN_SET
#elif	TS_On_state==0
	#define TS_state GPIO_PIN_RESET
#endif

#if KZBH_On_state
	#define KZBH_state GPIO_PIN_SET
#elif	KZBH_On_state==0
	#define KZBH_state GPIO_PIN_RESET
#endif

#if ADI_On_state
	#define ADI_state GPIO_PIN_SET
#elif ADI_On_state==0
	#define ADI_state GPIO_PIN_RESET
#endif

#if MIGMMA_On_state
	#define MIGMMA_state GPIO_PIN_SET
#elif	MIGMMA_On_state==0
	#define MIGMMA_state GPIO_PIN_RESET
#endif

#if MOTOIV_On_state
	#define MOTOIV_state GPIO_PIN_SET
#elif	MOTOIV_On_state==0
	#define MOTOIV_state GPIO_PIN_RESET
#endif

#if CCCV_On_state
	#define CCCV_state GPIO_PIN_SET
#elif	CCCV_On_state==0
	#define CCCV_state GPIO_PIN_RESET
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t PWMON_State=0;			//PWMON启动标志符
uint8_t FPWM_State=0;			//FPWM标志符
uint8_t DA_State=0;				//DA启动标识符
uint8_t IO_Set=0;				//IO配置标志符
uint8_t DA_Set=1;				//DA配置标志符
uint8_t PWM_Set=1;				//PWM配置标志符
uint16_t ST_SAMPL_IT=0;			//ST采样标志符
uint16_t ALLSTATE_Set=0;		//各状态初始标志位
uint8_t SA_State=0;				//SA状态标志位
uint8_t UP_edge_signal=0;		//SA上升沿判断				
uint8_t DOWN_edge_signal=0;		//SA下降沿判断
uint8_t Wave_control_signal=1;	//波形控制标识符
uint8_t Short_circuit_signal=0;
uint8_t Weld_Wire_Set=0;		//焊丝选取标志

//------------FB----------
uint16_t FB_FIEP[5]={0};		
uint16_t FB_FIEP_PACK[15]={0};		
volatile uint16_t FB_FIEP_NUM=0;	
uint16_t FB_OUTI[5]={0};
uint16_t FB_OUTI_PACK[15]={0};
volatile uint16_t FB_OUTI_NUM=0;
uint16_t FB_OUTV[5]={0};
uint16_t FB_OUTV_PACK[15]={0};
volatile uint16_t FB_OUTV_NUM=0;

uint16_t FB_MTRP[5]={0};
uint16_t FB_MTRP_NUM=0;
uint16_t FB_PWRV[5]={0};
uint16_t FB_PWRV_NUM=0;
uint16_t FB_TIIR[5]={0};
uint16_t FB_TIIR_NUM=0;
//------------ST-----------
uint16_t ST_FPWM[5]={0};		
volatile uint16_t ST_FPWM_NUM=0;		
uint16_t ST_EPWM[5]={0};
volatile uint16_t ST_EPWM_NUM=0;	
uint16_t ST_OTDA[5]={0};
volatile uint16_t ST_OTDA_NUM=0;		//O-DA set

uint16_t ST_TEMP_1=0;		//SET_SAMPLE_TEMP
uint16_t ST_TEMP_2=0;
uint16_t ST_TEMP_3=0;
uint16_t EPWM_TEMP=0;

uint16_t AOC_SAI=0;
uint16_t AOV_SAI=0;

volatile uint8_t FEED_SLOW_Signal=0;	//1->ON 0->OFF
volatile uint8_t FEED_Growth_Signal=0;		//1->ON 0->OFF
volatile uint16_t FB_FIEP_I=0;
volatile int16_t FB_FIEP_D=0;

uint8_t HQ_Detec_Back=0;
uint8_t JS_Detec_Back=0;
uint8_t ICOK_Detec_Back=0;

uint8_t HQ_temp[10]={0};
uint8_t JS_temp[10]={0};
uint8_t ICOK_temp[10]={0};
uint8_t HQ_state_temp=0;
uint8_t JS_state_temp=0;
uint8_t ICOK_state_temp=0;
uint8_t IOSP_IT=0;
uint8_t IOFB_IT=0;
uint8_t AVFB_IT=0;
volatile uint8_t res_count=0;

uint8_t SA_Prev=0;
uint16_t edge_count_it=0;

float Crt_div = 0;
float Vol_div = 0;
float Res_div = 0;

GAS_WELDING_DATA Gas_a;
uint16_t* GMAW_GAS_Diam_Speed=NULL;
uint16_t* GMAW_GAS_Diam_Volt=NULL;
uint16_t* GMAW_GAS_Diam_Vc=NULL;
uint8_t* GMAW_GAS_Diam_Inductor=NULL;
uint8_t* GMAW_GAS_Diam_VSLOPE=NULL;

uint8_t Error_Code=0;
uint16_t Error_Hold_count=0;

//uint8_t i2c_temp=0;
//HAL_StatusTypeDef i2c_state=0;
//uint8_t i2c_buffer_write[32]={0};
//uint8_t i2c_buffer_recive[32]={0};
//uint8_t uart_buffer_recive[32]={0};


uint16_t TM7_count_temp=0;
uint16_t State_Delay_count=0;
uint16_t Edge_Delay_count=0;
uint16_t State_Delay_contain=0;
uint8_t inter_arc_signal=0;

extern uint16_t FPWM_Set;

extern uint16_t GMAW_GAS_Speed[50];
extern uint8_t GMAW_GAS_Inductor[50];
extern uint16_t GMAW_GAS_Vc[50];
extern uint8_t GMAW_GAS_VSLOPE[50];
extern uint16_t GMAW_GAS_Volt[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void Get_FB_FUN(void);
void Get_SP_FUN(void);
void Switch_State_FUN(void);
void Get_State_FUN(void);
void Weld_ERROR_Handler(void);
void Test_FUN(void);
void IT_Respond_FUN(void);
void Uart_Handler_FUN(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GMAWStatus_t GMAW_State=GMAW_STANDBY;

TASK_COMPONENTS Task_Comps[]=		//10 us per/count
{
//状态  计数  周期  函数
	{0,1,4,Get_FB_FUN},
	{0,3,4,Switch_State_FUN},
	{0,100,100,Get_State_FUN},
	{0,500,500,Get_SP_FUN},
//	{0,1,1,IT_Respond_FUN}
//	{0,2000,2000,Uart_Handler_FUN}
	{0,15,15,Weld_ERROR_Handler}
};

uint8_t Tasks_Max = sizeof(Task_Comps)/sizeof(Task_Comps[0]);

void Task_Pro_Handler_Callback(void)
{
	uint8_t i;
	for(i=0; i<Tasks_Max; i++)
	{
		if(Task_Comps[i].Run) /* If task can be run */
		{
			Task_Comps[i].Run = 0;    /* Flag clear 0 */
			Task_Comps[i].TaskHook();  /* Run task */
		}
	}
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	IO_Set=1;
//	HAL_TIM_Base_Start_IT(&htim3);			//FPWM 启动
	FPWM_State=0;
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);			//  PWMON 启动
	PWMON_State=0;
	
	HAL_TIM_Base_Start_IT(&htim7);		//Switch Delay Detect Timer 7
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);			//  ELCPWM 启动
	htim1.Instance->CCR2=100;

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);		//AOV_SAI
	htim1.Instance->CCR3=0;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);		//AOC_SAI
	htim1.Instance->CCR4=0;
	

	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart(&hadc2);
	/*--------------------ADC CONTROLL SCHEDULE-------------------------------------
	ADC1:		Regular Conv:	
												RANK 1 :PA0 -> FIEP
					Inject	Conv:
												RANK 1 :PA1 -> L
												RANK 2 :PA2 -> I
												RANK 3: PC3	-> V

	ADC2:		Regular Conv:	
												RANK 1 :PC4 -> OUTV2
					Inject	Conv:
												RANK 1 :PC5 -> OUTI2
												RANK 2 :PA1 ->MTRP
												RANK 3 :PA2 ->PWRV
												RANK 4 :PC3 ->TIIR
	
	Renewal in 2024/4/9
	------------------------------------------------------------------------------*/
	
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,DAC1_Value);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
	
	
	if(IO_Set)			//IO Setting
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,TS_state);		//TS
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,KZBH_state);		//KZBH
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,ADI_state);		//A/D-L
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,MIGMMA_state);		//MIGMMA
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,CCCV_state);	
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	//ARCF
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//HSW
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//BBR
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);	//MDFY
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);	//VGEAR
		
		IO_Set--;			//configure in 1 times
	}
	
	ALLSTATE_Set ^=0x0001;
		
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		Task_Pro_Handler_Callback();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
 
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 35;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 11;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 35;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTO_IV_Pin|ARCF_Pin|HSW_Pin|BBR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SYNOUT_Pin|PWM_ON_Pin|MIG_MMA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SMPL_Pin|FPWM_Pin|VGEAR_Pin|CC_CV_Pin
                          |A_D_L_Pin|TS_Pin|KZBH_Pin|ARC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WD_Pin IC_OK_Pin PC6 FEED_Pin */
  GPIO_InitStruct.Pin = WD_Pin|IC_OK_Pin|GPIO_PIN_6|FEED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTO_IV_Pin ARCF_Pin HSW_Pin BBR_Pin */
  GPIO_InitStruct.Pin = MOTO_IV_Pin|ARCF_Pin|HSW_Pin|BBR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TANK_Pin MDFY_Pin */
  GPIO_InitStruct.Pin = TANK_Pin|MDFY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNOUT_Pin PWM_ON_Pin MIG_MMA_Pin */
  GPIO_InitStruct.Pin = SYNOUT_Pin|PWM_ON_Pin|MIG_MMA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPL_Pin FPWM_Pin VGEAR_Pin CC_CV_Pin
                           A_D_L_Pin TS_Pin KZBH_Pin ARC_Pin */
  GPIO_InitStruct.Pin = SMPL_Pin|FPWM_Pin|VGEAR_Pin|CC_CV_Pin
                          |A_D_L_Pin|TS_Pin|KZBH_Pin|ARC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MISS_Pin PB10 PB11 ER4_Pin
                           ER2_Pin ER1_Pin ER0_Pin */
  GPIO_InitStruct.Pin = MISS_Pin|GPIO_PIN_10|GPIO_PIN_11|ER4_Pin
                          |ER2_Pin|ER1_Pin|ER0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SA_Pin */
  GPIO_InitStruct.Pin = SA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_Pin */
  GPIO_InitStruct.Pin = START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Get_FB_FUN(void)
{	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	
	
	uint8_t i,j,index=0;
	uint16_t	FB_FIEP_TEMP;	
	uint16_t	FB_OUTI_TEMP;
	uint16_t 	FB_OUTV_TEMP;
	
	for(i=0;i<5;i++)		//Capture FIEP Analoge
	{
		FB_FIEP[i]=HAL_ADC_GetValue(&hadc1);
		FB_OUTI[i]=HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
		FB_OUTV[i]=HAL_ADC_GetValue(&hadc2);		
	}
		
	
	FB_FIEP_NUM=(FB_FIEP[0]+FB_FIEP[1]+FB_FIEP[2]+FB_FIEP[3]+FB_FIEP[4])/5;		//Average Cap
	FB_OUTI_NUM=(FB_OUTI[0]+FB_OUTI[1]+FB_OUTI[2]+FB_OUTI[3]+FB_OUTI[4])/5;		//Average Set
	FB_OUTV_NUM=(FB_OUTV[0]+FB_OUTV[1]+FB_OUTV[2]+FB_OUTV[3]+FB_OUTV[4])/5;	
	
	FB_MTRP_NUM=HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_2);
	FB_PWRV_NUM=HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_3);
	FB_TIIR_NUM=HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_4);
	
	//----------Average FeedBack function-----------
	FB_FIEP_TEMP=0;	
	FB_OUTI_TEMP=0;
	FB_OUTV_TEMP=0;
		
	for(i=0;i<14;i++)
	{
		FB_FIEP_PACK[14-i]=FB_FIEP_PACK[14-i-1];
		FB_FIEP_TEMP+=FB_FIEP_PACK[14-i];							//TEMP:Sum & Average num
		FB_OUTI_PACK[14-i]=FB_OUTI_PACK[14-i-1];
		FB_OUTI_TEMP+=FB_OUTI_PACK[14-i];
		FB_OUTV_PACK[14-i]=FB_OUTV_PACK[14-i-1];
		FB_OUTV_TEMP+=FB_OUTV_PACK[14-i];
	}		
	FB_FIEP_PACK[0]=FB_FIEP[0];
	FB_OUTI_PACK[0]=FB_OUTI[0];
	FB_OUTV_PACK[0]=FB_OUTV[0];
	
	FB_FIEP_TEMP=(FB_FIEP_PACK[0]+FB_FIEP_TEMP)/15;		//Average FIEP in 15 times
	FB_FIEP_I=FB_FIEP_TEMP;
	FB_OUTI_TEMP=(FB_OUTI_PACK[0]+FB_OUTI_TEMP)/15;
	AOC_SAI=FB_OUTI_TEMP;
	FB_OUTV_TEMP=(FB_OUTV[0]+FB_OUTV_TEMP)/15;
	AOV_SAI=FB_OUTV_TEMP;
	
	FB_FIEP_D=FB_FIEP_PACK[0]-FB_FIEP_PACK[1];	//D
	
	
	ICOK_state_temp-=ICOK_temp[IOFB_IT];
			
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==GPIO_PIN_SET)		//ICOK
		ICOK_temp[IOFB_IT]=1;
	else
		ICOK_temp[IOFB_IT]=0;
		
	ICOK_state_temp+=ICOK_temp[IOFB_IT];	
	IOFB_IT=(IOFB_IT+1)%10;
	if(ICOK_state_temp>6){
		ICOK_Detec_Back=1;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	else{
		ICOK_Detec_Back=0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	}
	
	
	
	/* SA Function  */
	if(Wave_control_signal==1&&GMAW_State==GMAW_STABILIZE){
		if(FB_OUTV_NUM*5/4<Gas_a.weld_volt-650){		//750+BBR 	650
			if(SA_Prev==1){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			
				DOWN_edge_signal=0;		//ecl				
				UP_edge_signal=1;
				edge_count_it=0;
			}
			SA_Prev=0;
		}
		else if(FB_OUTV_NUM*5/4>Gas_a.weld_volt-500){
			if(SA_Prev==0){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
				DOWN_edge_signal=1;		//Correct Vol_up				
				UP_edge_signal=0;
				edge_count_it=0;
			}
			SA_Prev=1;
		}
		else NULL;
	}
	
	
	if(GMAW_State==GMAW_STABILIZE)
	{
		
		if(DOWN_edge_signal==1){
			//VOL PLUS
			if(FB_OUTI_NUM>10){
				if(edge_count_it<2)      //10-》4-》2--3.25
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt);
				else {
					if(edge_count_it*Gas_a.weld_vslope+Gas_a.weld_volt>=2*Gas_a.weld_vslope+Gas_a.weld_vc+Gas_a.weld_volt+300) //
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt-300);		
					else{
						if(Gas_a.weld_volt>Gas_a.weld_vc+600+ST_FPWM_NUM/4)
							HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt+Gas_a.weld_vc-(edge_count_it-2)*Gas_a.weld_vslope);
						else
							HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
					}
				}
				Short_circuit_signal=0;
			}
			else{
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,2100);
				Short_circuit_signal=1;
			}
			
			//BBR 
//			if(edge_count_it>(220-Gas_a.weld_vslope*8) && Gas_a.weld_vslope>0 &&  Short_circuit_signal==0){
//				if((edge_count_it+1)%10==0)
//					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//				if(edge_count_it%10==0)
//					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
//			}
			
			edge_count_it++;	
		}
		
		if(UP_edge_signal==1){
			//ECL PLUS
			if(edge_count_it<10)       //10-》5-》10  3.25 3.26
					edge_count_it++;
			else{
				if(Gas_a.weld_Inductor>10){     //40->20  3.26
					EPWM_TEMP=Gas_a.weld_Inductor*5/8;    //40->20  3.26
					htim1.Instance->CCR2=EPWM_TEMP;
				}
				else{
					EPWM_TEMP=0;
					htim1.Instance->CCR2=EPWM_TEMP;
				}
				UP_edge_signal=0;
				edge_count_it=0;
			}
			
		}
	
	}
	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);	
}


void Get_SP_FUN()
{
	uint16_t ST_EPWM_ADJUST=0;
	uint16_t ST_FPWM_ADJUST=0;
	uint16_t ST_EPWM_Container=0;
	
	ST_EPWM[ST_SAMPL_IT]=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);//L_Cai
	ST_FPWM[ST_SAMPL_IT]=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);//I_Cai
	ST_OTDA[ST_SAMPL_IT]=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);//V_Cai

	ST_SAMPL_IT=(ST_SAMPL_IT+1)%5;
	
	ST_EPWM_NUM=(ST_EPWM[0]+ST_EPWM[1]+ST_EPWM[2]+ST_EPWM[3]+ST_EPWM[4])/5;//L_Cai
	ST_OTDA_NUM=(ST_OTDA[0]+ST_OTDA[1]+ST_OTDA[2]+ST_OTDA[3]+ST_OTDA[4])/5;//V_Cai
	ST_FPWM_NUM=(ST_FPWM[0]+ST_FPWM[1]+ST_FPWM[2]+ST_FPWM[3]+ST_FPWM[4])/5;//I_Cai	
	
	
	if(ST_FPWM_NUM>365)
		ST_FPWM_ADJUST=(ST_FPWM_NUM/8+4.325)/2-25;
	else
		ST_FPWM_ADJUST=0;
	if(ST_FPWM_ADJUST>196) ST_FPWM_ADJUST=196;
	

//	ST_FPWM_ADJUST=52;
	
	//take array form datesheet
	if(0){
		Gas_a.weld_speed=
		GMAW_GAS_Speed[ST_FPWM_ADJUST/4]+(GMAW_GAS_Speed[ST_FPWM_ADJUST/4+1]-GMAW_GAS_Speed[ST_FPWM_ADJUST/4])*(ST_FPWM_ADJUST%4)/4;
		Gas_a.weld_volt=
		GMAW_GAS_Volt[ST_FPWM_ADJUST/4]+(GMAW_GAS_Volt[ST_FPWM_ADJUST/4+1]-GMAW_GAS_Volt[ST_FPWM_ADJUST/4])*(ST_FPWM_ADJUST%4)/4;
		ST_EPWM_Container=GMAW_GAS_Inductor[ST_FPWM_ADJUST/4];
		Gas_a.weld_vc=GMAW_GAS_Vc[ST_FPWM_ADJUST/4];
		Gas_a.weld_vslope=GMAW_GAS_VSLOPE[ST_FPWM_ADJUST/4];
		
		
		if(ST_OTDA_NUM>=1138)		//4.15 845->1138
			Gas_a.weld_volt=Gas_a.weld_volt+(ST_OTDA_NUM-1138)*(ST_FPWM_ADJUST*0.01+0.25);
		else{
			if(Gas_a.weld_volt>1138-ST_OTDA_NUM)
				Gas_a.weld_volt=Gas_a.weld_volt-(1138-ST_OTDA_NUM)*(ST_FPWM_ADJUST*0.01+0.25);
			else
				Gas_a.weld_volt=0;
		}	
		
		if(ST_EPWM_NUM>2000){
			ST_EPWM_ADJUST=(ST_EPWM_NUM-2000)/20;
			Gas_a.weld_Inductor=ST_EPWM_Container+ST_EPWM_ADJUST;
			if(Gas_a.weld_Inductor>200) Gas_a.weld_Inductor=200;
		}
		else{
			ST_EPWM_ADJUST=(2000-ST_EPWM_NUM)/20;
			if(ST_EPWM_ADJUST>ST_EPWM_Container)
				Gas_a.weld_Inductor=0;
			else
				Gas_a.weld_Inductor=ST_EPWM_Container-ST_EPWM_ADJUST;
		}
		
	}
	
	if(1)
	{
		/******************************
		PB10 ->LOW		PB11 ->HIGH		PC6->SYN_IN
		0				0				0
		
			
		000->	CO2_12
		001->	CO2_10
		010->	CO2_16
		011->	MAG_10
		100->	MAG_12
		101->	MAG_16
		110->	FLUX_12
		111->	FLUX_16
		
		******************************/
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_RESET)	Weld_Wire_Set |= 0b00000100;	else	Weld_Wire_Set &= 0b11111011;
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==GPIO_PIN_RESET)	Weld_Wire_Set |= 0b00000010;	else	Weld_Wire_Set &= 0b11111101;
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)==GPIO_PIN_RESET)	Weld_Wire_Set |= 0b00000001;	else	Weld_Wire_Set &= 0b11111110;
	
		switch(Weld_Wire_Set)
		{
			case 0:
				GMAW_GAS_Diam_Speed=GMAW_Wave_CO2_12_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_CO2_12_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_CO2_12_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_CO2_12_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_CO2_12_VSLOPE;
				break;
			case 1:
				GMAW_GAS_Diam_Speed=GMAW_Wave_CO2_10_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_CO2_10_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_CO2_10_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_CO2_10_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_CO2_10_VSLOPE;
				break;
			case 2:
				GMAW_GAS_Diam_Speed=GMAW_Wave_CO2_16_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_CO2_16_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_CO2_16_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_CO2_16_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_CO2_16_VSLOPE; 
				break;
			case 3:
				GMAW_GAS_Diam_Speed=GMAW_Wave_MAG_10_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_MAG_10_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_MAG_10_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_MAG_10_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_MAG_10_VSLOPE; 
				break;
			case 4:
				GMAW_GAS_Diam_Speed=GMAW_Wave_MAG_12_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_MAG_12_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_MAG_12_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_MAG_12_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_MAG_12_VSLOPE; 
				break;
			case 5:
				GMAW_GAS_Diam_Speed=GMAW_Wave_MAG_16_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Wave_MAG_16_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Wave_MAG_16_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Wave_MAG_16_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Wave_MAG_16_VSLOPE; 
				break;
			case 6:
				GMAW_GAS_Diam_Speed=GMAW_Flux_CO2_12_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Flux_CO2_12_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Flux_CO2_12_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Flux_CO2_12_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Flux_CO2_12_VSLOPE; 
				break;
			case 7:
				GMAW_GAS_Diam_Speed=GMAW_Flux_CO2_16_Speed;
				GMAW_GAS_Diam_Volt=GMAW_Flux_CO2_16_Volt;
				GMAW_GAS_Diam_Vc=GMAW_Flux_CO2_16_Vc;
				GMAW_GAS_Diam_Inductor=GMAW_Flux_CO2_16_Inductor;
				GMAW_GAS_Diam_VSLOPE=GMAW_Flux_CO2_16_VSLOPE;
				break;
		}
		
		Gas_a.weld_speed= 
		GMAW_GAS_Diam_Speed[ST_FPWM_ADJUST/4]+(GMAW_GAS_Diam_Speed[ST_FPWM_ADJUST/4+1]-GMAW_GAS_Diam_Speed[ST_FPWM_ADJUST/4])*(ST_FPWM_ADJUST%4)/4;
		Gas_a.weld_volt=
		GMAW_GAS_Diam_Volt[ST_FPWM_ADJUST/4]+(GMAW_GAS_Diam_Volt[ST_FPWM_ADJUST/4+1]-GMAW_GAS_Diam_Volt[ST_FPWM_ADJUST/4])*(ST_FPWM_ADJUST%4)/4;
		ST_EPWM_Container=GMAW_GAS_Diam_Inductor[ST_FPWM_ADJUST/4];
		Gas_a.weld_vc=GMAW_GAS_Diam_Vc[ST_FPWM_ADJUST/4];
		Gas_a.weld_vslope=GMAW_GAS_Diam_VSLOPE[ST_FPWM_ADJUST/4];
		
		
		if(ST_OTDA_NUM>=1138)		//4.15 845->1138
			Gas_a.weld_volt=Gas_a.weld_volt+(ST_OTDA_NUM-1138)*(ST_FPWM_ADJUST*0.005+0.25);
		else{
			if(Gas_a.weld_volt>1138-ST_OTDA_NUM)
				Gas_a.weld_volt=Gas_a.weld_volt-(1138-ST_OTDA_NUM)*(ST_FPWM_ADJUST*0.005+0.25);
			else
				Gas_a.weld_volt=0;
		}
		
		if(ST_EPWM_NUM>2000){
			ST_EPWM_ADJUST=(ST_EPWM_NUM-2000)/20;
			Gas_a.weld_Inductor=ST_EPWM_Container+ST_EPWM_ADJUST;
			if(Gas_a.weld_Inductor>200) Gas_a.weld_Inductor=200;
		}
		else{
			ST_EPWM_ADJUST=(2000-ST_EPWM_NUM)/20;
			if(ST_EPWM_ADJUST>ST_EPWM_Container)
				Gas_a.weld_Inductor=0;
			else
				Gas_a.weld_Inductor=ST_EPWM_Container-ST_EPWM_ADJUST;
		}
	
	}
	
	
	
	
	// I/O Detec	
	HQ_state_temp-=HQ_temp[IOSP_IT];
	JS_state_temp-=JS_temp[IOSP_IT];
		
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)==GPIO_PIN_RESET)		//HQ
		HQ_temp[IOSP_IT]=1;
	else 
		HQ_temp[IOSP_IT]=0;
	
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)==GPIO_PIN_RESET)		//JS
		JS_temp[IOSP_IT]=1;	
	else 
		JS_temp[IOSP_IT]=0;
			
	HQ_state_temp+=HQ_temp[IOSP_IT];
	JS_state_temp+=JS_temp[IOSP_IT];
	
	IOSP_IT=(IOSP_IT+1)%10;
		
	if(HQ_state_temp>6)			
		HQ_Detec_Back=1;
	else
		HQ_Detec_Back=0;
		
	if(JS_state_temp>6)
		JS_Detec_Back=1;
	else
		JS_Detec_Back=0;
	
	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_SET){		//Wave Switch  	
		if(Wave_control_signal==1) Wave_control_signal=0;
	}
	else{														//不接线
		if(Wave_control_signal==0) Wave_control_signal=1;
	}
	
}

void Switch_State_FUN()
{
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	
	
	switch(GMAW_State)
		{
		//----------------------待机状态/0
		case GMAW_STANDBY:	
			
			if((ALLSTATE_Set & 0x0001) != 0){
				State_Delay_count=0;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);		//HSW_OFF	
				ALLSTATE_Set ^= 0x0001;
			}
			
			if(DA_State==1){
//				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);												
				DA_State=0;
			}
			if(FPWM_State==1){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim6);
				FPWM_State=0;
			}
//			if(FPWM_State==0){
//				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
//				HAL_TIM_Base_Start_IT(&htim6);
//				FPWM_State=1;
//			}
			
			//检丝
			if(JS_Detec_Back==1){
				if(FPWM_State==0){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
					HAL_TIM_Base_Start_IT(&htim6);
					FPWM_State=1;
				}				
			}
			if(JS_Detec_Back==0){
				if(FPWM_State==1){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
					HAL_TIM_Base_Stop_IT(&htim6);
					FPWM_State=0;
				}
			}
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+1600);
			
			//ARC RESET
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);		
			
			State_Delay_count++;
						
			if(HQ_Detec_Back==1){
				ALLSTATE_Set = 0x0002;
				GMAW_State++;
			}
			
				break;
		//----------------------逆变启动/1
		case GMAW_INVERTER_ON:		
			if((ALLSTATE_Set & 0x0002) != 0){	//状态初始化
				State_Delay_count=0;
				ALLSTATE_Set ^= 0x0002;
			}
		
			if(DA_State==0)											
				DA_State=1;
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+1600);
			
//			State_Delay_count++;
//			if(State_Delay_count>=1000)
				GMAW_State++;
			
			
			if(HQ_Detec_Back==0)	GMAW_State=GMAW_STANDBY;	//焊枪检测
				break;
		//----------------------慢送丝/2
		case GMAW_RUN_IN:
			if((ALLSTATE_Set & 0x0004) != 0){
				ALLSTATE_Set ^= 0x0004;
			}
		
			if(FPWM_State==0){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
				HAL_TIM_Base_Start_IT(&htim6);
				FPWM_State=1;
			}
			if(ICOK_Detec_Back==0)
				FEED_SLOW_Signal=1;
			else
				FEED_SLOW_Signal=0;
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+1600);		//ARC
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			if(HQ_Detec_Back==0)	GMAW_State=GMAW_STANDBY;
			GMAW_State++;
				break;
			//----------------------ARC 检测/3	--------tip:空载状态
		case GMAW_ARC_DETECT:
			if((ALLSTATE_Set & 0x0008) != 0){
				ALLSTATE_Set ^= 0x0008;
			}
			
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==SET&&FB_OUTV_NUM>=100&&FB_OUTI_NUM>=100)	//IC_OK/FB_I/FB_V
			{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);	//ARC signal C->S
				ALLSTATE_Set ^=0x0010;
				GMAW_State++;
			}
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+1600);
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			if(HQ_Detec_Back==0)  GMAW_State=GMAW_STANDBY;
			break;
		//----------------------高压引弧_热引弧/4
		case GMAW_FLASH_START:
			if((ALLSTATE_Set & 0x0010) != 0){
				
				State_Delay_count=0;
				State_Delay_contain=0;
				inter_arc_signal=0;
				
				ALLSTATE_Set ^= 0x0010;
			}
			
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+1600);		//ARC
			

			if((State_Delay_count<100+Gas_a.weld_volt/2)&&(inter_arc_signal==0)){
				State_Delay_count++;
				State_Delay_contain=State_Delay_count;
			}
			if((State_Delay_count==100+Gas_a.weld_volt/2)||(FB_OUTI_NUM<1200))
				inter_arc_signal=1;
			if(inter_arc_signal==1){
				State_Delay_count++;
				if(State_Delay_count-State_Delay_contain >= 150){
					if(FB_OUTI_NUM<200){
						State_Delay_count=0;
						inter_arc_signal=0;
						
						
						State_Delay_contain=0;
					}
					else{
						GMAW_State++;
						ALLSTATE_Set ^= 0x0020;
						State_Delay_count=0;
						inter_arc_signal=0;
					}
				}
			}
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			if(HQ_Detec_Back==0)  GMAW_State=GMAW_STANDBY;
			break;
		//----------------------建立稳定熔池/5
		case GMAW_STABILIZE:
			if((ALLSTATE_Set & 0x0020) != 0){
						
				if(FEED_Growth_Signal==0){
					FEED_Growth_Signal=1;
					FPWM_Set=400;
				}
				
				if(FEED_SLOW_Signal==1)		
					FEED_SLOW_Signal=0;	
				
				UP_edge_signal=0;
				DOWN_edge_signal=0;
				SA_State=0;
				Edge_Delay_count=0;
				Wave_control_signal=2;
				
				State_Delay_count=0;
				ALLSTATE_Set ^= 0x0020;
			}
			
	
			//ECL PLUS restore
			if(UP_edge_signal==0){
				EPWM_TEMP=Gas_a.weld_Inductor;
				htim1.Instance->CCR2=EPWM_TEMP;
			}
			
			//Wave Control Delay
			if(State_Delay_count<4000) 
				State_Delay_count++;
			else if(State_Delay_count==4000){
				Wave_control_signal=0;
				State_Delay_count++;
			}
			else
				NULL;
			
			
//			//Resist display 
//			Vol_div=FB_OUTV_NUM;
//			Crt_div=FB_OUTI_NUM;
//			Res_div=Vol_div/Crt_div*500;
//			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint16_t)Res_div);
			
			//Vol PLUS	restore	
			if(DOWN_edge_signal==0 && Short_circuit_signal==0){
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//deleta
			}				
			
//			//TEMPER Detec
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET)
//				GMAW_State=GMAW_ERROR;
			
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			if(HQ_Detec_Back==0)  GMAW_State++;
			break;
		//----------------------点焊时暂停电机/6
		case GMAW_DWELL:
			if((ALLSTATE_Set & 0x0040) != 0){
				ALLSTATE_Set ^= 0x0040;
			}
		
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			GMAW_State++;
			break;
		//----------------------结束阶段	/7	
		case GMAW_END_STAGE:
			if((ALLSTATE_Set & 0x0080) != 0){
				ALLSTATE_Set ^= 0x0080;
			}	
		
		
			if(FPWM_State==1){		//关闭电机
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim6);
				FPWM_State=0;
			}
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			ALLSTATE_Set ^= 0x0100;
			GMAW_State++;
			break;
			//----------------------电机停止时降低输出电压/8   bug_log:概率卡死在次状态
		case GMAW_BURN_BACK:
			if((ALLSTATE_Set & 0x0100) != 0){
				State_Delay_count=0;
				ALLSTATE_Set ^= 0x0100;
			}
		
			State_Delay_count++;
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Gas_a.weld_volt/4+500);
			
			//DA heart beat
			if(DA_State==1)
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
			
			if(State_Delay_count>=1000+Gas_a.weld_speed/2)
				GMAW_State=GMAW_STANDBY;
			break;
		//----------------------电流冲击FFT	/9
		case GMAW_WIRE_SHARP:
			if((ALLSTATE_Set & 0x0200) != 0){
				ALLSTATE_Set ^= 0x0200;
			}
		
			break;
		//----------------------焊接故障/10
		case GMAW_ERROR:
			if((ALLSTATE_Set & 0x0400) != 0){
				ALLSTATE_Set ^= 0x0400;
			}
			
			if(DA_State==1){											
				DA_State=0;
			}
			if(FPWM_State==1){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim6);
				FPWM_State=0;
			}
			
			
			switch(Error_Code)
			{
				case 1:		//0001
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 2:		//0010
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 3:		//0011
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 4:		//0100
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 5:		//0101
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 6:		//0110
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
				case 7:		//0111
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
					break;
			}
			
	
			
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_RESET)
//				GMAW_State=GMAW_STANDBY;
			
			break;
		}	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);		
}


void Get_State_FUN(void)
{
	
	if(PWM_Set)			//PWM Setting
		{
			if(GMAW_State!=GMAW_STABILIZE)
			{
				EPWM_TEMP=Gas_a.weld_Inductor;			
				htim1.Instance->CCR2=EPWM_TEMP;
			}
			
			htim1.Instance->CCR3=AOV_SAI/24;
			htim1.Instance->CCR4=AOC_SAI/24;
		}
				
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GMAW_State==GMAW_STABILIZE&&Wave_control_signal==1){
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_SET)
			if(FB_OUTV_NUM>700+ST_FPWM_NUM/8&&Short_circuit_signal==0)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//ARCF SET 
//					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//BBR SET 
					res_count=1;
				}

		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == GPIO_PIN_SET){
			DOWN_edge_signal=0;
			Short_circuit_signal=0;
			if(UP_edge_signal==0)
				edge_count_it=0;
		
		}
			
	}

}

void IT_Respond_FUN(void)
{
	if(res_count==1)
		res_count++;
	else if(res_count==2){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		res_count=0;
	}
		
}

void Uart_Handler_FUN(void)
{
	uint8_t i_sum=0;
	uint32_t i2c_error=0;
	
//	for(int i=0;i<8;i++)
//		i2c_buffer_write[i]=i+2;
//	
	/*
		I2C device type identifier "1010"(A hex) in bits 7 through 4   ---> 1010 0000  =0xA0
			
		AT24C04C Memory  32 pages of 16 Bytes each
		AT24C08C Memory  64 pages of 16 Bytes each
	
	*/
	//  1.I2C设备号指针  2.从设备(E2R)地址  3.从机寄存器起始地址 4.从机寄存器地址长度8b/16b 5.需写入数据起始地址 6.Size 7.TimeOut
//	if(i2c_temp==0)
//		i2c_state=HAL_I2C_Mem_Write(&hi2c2,0xA0,i_sum,I2C_MEMADD_SIZE_8BIT,&(i2c_buffer_write[i_sum]),8,0);
//	
//	i_sum=0;
//	if(i2c_temp==1)
//		if(i2c_state==HAL_OK)
//			i2c_state=HAL_I2C_Mem_Read(&hi2c2,0xA1,i_sum,I2C_MEMADD_SIZE_8BIT,&(i2c_buffer_recive[i_sum]),8,0);
//	
//	if(i2c_state==HAL_ERROR)
//		i2c_error=hi2c2.ErrorCode;
//	
//	i2c_temp=(i2c_temp+1)%2;
//	
	
	if(1)
		Wave_control_signal=1;
	
}

void Weld_ERROR_Handler(void)
{
	
	/*		温度保护		*/
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET){
		GMAW_State=GMAW_ERROR;
		Error_Code=1;
	}
	else{
		if(Error_Code==1){
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		缺相保护		*/
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==GPIO_PIN_SET){
		GMAW_State=GMAW_ERROR;
		Error_Code=2;
	}
	else{
		if(Error_Code==2){
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		过压保护		*/
	if(FB_TIIR_NUM>3330){
		GMAW_State=GMAW_ERROR;
		Error_Code=3;
	}
	else{
		if(Error_Code==3){ 
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		欠压保护		*/
	if(FB_TIIR_NUM<2172&&DA_State==1){
		GMAW_State=GMAW_ERROR;
		Error_Code=4;
	}
	else{
		if(Error_Code==4){
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		MTRP		*/
	if(FB_MTRP_NUM>2860){
		GMAW_State=GMAW_ERROR;
		Error_Code=5;
	}
	else{
		if(Error_Code==5){
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		PWRV		*/
	if(FB_PWRV_NUM>2800||FB_PWRV_NUM<1800){
		GMAW_State=GMAW_ERROR;
		Error_Code=6;
	}
	else{
		if(Error_Code==6){ 
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
		}
	}
	/*		电流保护 */
	if(FB_OUTI_NUM>3232){
		if(Error_Hold_count>=1200){		//200ms 400*500us
			GMAW_State=GMAW_ERROR;
			Error_Hold_count=0;
			Error_Code=7;
		}
		else
			Error_Hold_count++;
	}
	else{
		if(Error_Hold_count>0)	
			Error_Hold_count--;
		else
			Error_Hold_count=0;			
	}
	if(Error_Hold_count<20&&Error_Code==7){
		Error_Code=0;
		GMAW_State=GMAW_STANDBY;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
