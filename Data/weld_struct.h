#ifndef __WELD_STRUCT_H
#define __WELD_STRUCT_H


#include "stm32f1xx_hal.h"

/*  WELDING DATA  */

uint16_t GMAW_GAS_Speed[200];		//60-500 速度
uint16_t GMAW_GAS_Volt[125];		//30-250 电压
uint16_t GMAW_GAS_Inductor[200];	//		 电感

uint8_t GMAW_GAS_Vc[100];			//Vol_Plus 电压变化幅限
uint8_t GMAW_GAS_VSLOPE[100];		//Vol_plus 电压变化斜率

uint8_t GMAW_GAS_Varc[10];			//ARC 时间
uint8_t GMAW_GAS_Bbk[10];			//收弧 时间

typedef struct {
	uint16_t weld_speed;
	uint16_t weld_volt;
	uint16_t weld_Inductor;


}GAS_WELDING_DATA;





/* UART DATA */

typedef struct
{
	uint8_t BTN_OP1;
	uint8_t BTN_OP2;
	uint8_t BTN_JOB;
	uint8_t BTN_ECL;
	uint16_t BTN_CRT;
	uint16_t BTN_VOL;
	uint8_t BTN_ENDI;
	uint8_t BTN_ENDV;
	uint8_t BTN_ARCI;
	uint8_t BTN_ARCV;
}OP_SET;		//操作设定

typedef struct
{
	uint8_t SET_SLOWFD;
	uint8_t ARC_SET;
	uint8_t REVISE_SET;
	uint8_t GAS_SET;
	uint8_t PULSE_FREQ_SET;
	uint8_t PULSE_PWM_SET;
	uint8_t BASE_CRT;
	uint8_t BASE_VOL;
	uint8_t CONTROL_SET;
	uint8_t ADR_BPS;
}MENU_SET;

typedef struct
{
	uint8_t INTER_BASE_SET;
	uint8_t GENERATE_SET;
}INTER_MENU;

typedef struct
{
	uint8_t CPU_VERSION_NUMBER;
}OTHER_PRAMA;























#endif