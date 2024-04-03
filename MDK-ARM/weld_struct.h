#ifndef __WELD_STRUCT_H
#define __WELD_STRUCT_H


#include "stm32f1xx_hal.h"

/*  WELDING DATA  */

const uint16_t GMAW_GAS_Speed[50];		//60-500 �ٶ�
const uint16_t GMAW_GAS_Volt[50];		//30-250 ��ѹ
const uint8_t GMAW_GAS_Inductor[50];	//		 ���

const uint16_t GMAW_GAS_Vc[50];			//Vol_Plus ��ѹ�仯����
const uint8_t GMAW_GAS_VSLOPE[50];		//Vol_plus ��ѹ�仯б��

const uint8_t GMAW_GAS_Varc[10];			//ARC ʱ��
const uint8_t GMAW_GAS_Bbk[10];			//�ջ� ʱ��


//�޲��ο���  CO2
//const uint16_t GMAW_Flat_CO2_Speed[200];
//const uint8_t GMAW_Flat_CO2_Inductor[50];
////const uint16_t GMAW_Flat_CO2_Vc[50];
////const uint8_t GMAW_Flat_CO2_VSLOPE[50];

////���ο��� CO2
//const uint16_t GMAW_Wave_CO2_12_Speed[200];
//const uint8_t  GMAW_Wave_CO2_12_Inductor[50];
//const uint16_t GMAW_Wave_CO2_12_Vc[50];
//const uint8_t GMAW_Wave_CO2_12_VSLOPE[50];

////�޲��ο��� MAG
//const uint16_t GMAW_Flat_MAG_12_Speed[200];
//const uint8_t GMAW_Flat_MAG_12_Inductor[50];
////const uint16_t GMAW_Flat_MAG_Vc[50];
////const uint8_t GMAW_Flat_MAG_VSLOPE[50];

////���ο��� MAG
//const uint16_t GMAW_Flat_MAG_12_Speed[200];
//const uint8_t GMAW_Flat_MAG_12_Inductor[50];
//const uint16_t GMAW_Flat_MAG_12_Vc[50];
//const uint8_t GMAW_Flat_MAG_12_VSLOPE[50];


/*  CO2  1.2   */
const uint16_t GMAW_Wave_CO2_10_Speed[50];
const uint16_t GMAW_Wave_CO2_10_Volt[50];
const uint8_t  GMAW_Wave_CO2_10_Inductor[50];
const uint16_t GMAW_Wave_CO2_10_Vc[50];
const uint8_t  GMAW_Wave_CO2_10_VSLOPE[50];

/* CO2 1.4  */
const uint16_t GMAW_Wave_CO2_12_Speed[50];
const uint16_t GMAW_Wave_CO2_12_Volt[50];
const uint8_t  GMAW_Wave_CO2_12_Inductor[50];
const uint16_t GMAW_Wave_CO2_12_Vc[50];
const uint8_t  GMAW_Wave_CO2_12_VSLOPE[50];

/* CO2 1.6 */
const uint16_t GMAW_Wave_CO2_16_Speed[50];
const uint16_t GMAW_Wave_CO2_16_Volt[50];
const uint8_t  GMAW_Wave_CO2_16_Inductor[50];
const uint16_t GMAW_Wave_CO2_16_Vc[50];
const uint8_t  GMAW_Wave_CO2_16_VSLOPE[50];

/*	MAG 1.2	*/
const uint16_t GMAW_Wave_MAG_10_Speed[50];
const uint16_t GMAW_Wave_MAG_10_Volt[50];
const uint8_t  GMAW_Wave_MAG_10_Inductor[50];
const uint16_t GMAW_Wave_MAG_10_Vc[50];
const uint8_t  GMAW_Wave_MAG_10_VSLOPE[50];

/* MAG	1.4 */
const uint16_t GMAW_Wave_MAG_12_Speed[50];
const uint16_t GMAW_Wave_MAG_12_Volt[50];
const uint8_t  GMAW_Wave_MAG_12_Inductor[50];
const uint16_t GMAW_Wave_MAG_12_Vc[50];
const uint8_t  GMAW_Wave_MAG_12_VSLOPE[50];

/* MAG 1.6 */
const uint16_t GMAW_Flat_MAG_16_Speed[50];
const uint16_t GMAW_Wave_MAG_16_Volt[50];
const uint8_t  GMAW_Flat_MAG_16_Inductor[50];
const uint16_t GMAW_Flat_MAG_16_Vc[50];
const uint8_t  GMAW_Flat_MAG_16_VSLOPE[50];

/*	FLUX CO2 1.2 */
const uint16_t GMAW_Flux_CO2_12_Speed[50];
const uint16_t GMAW_Flux_CO2_12_Volt[50];
const uint8_t GMAW_Flux_CO2_12_Inductor[50];
const uint16_t GMAW_Flux_CO2_12_Vc[50];
const uint8_t GMAW_Flux_CO2_12_VSLOPE[50];

/*	FLUX CO2 1.6 */
const uint16_t GMAW_Flux_CO2_16_Speed[50];
const uint16_t GMAW_Flux_CO2_16_Volt[50];
const uint8_t GMAW_Flux_CO2_16_Inductor[50];
const uint16_t GMAW_Flux_CO2_16_Vc[50];
const uint8_t GMAW_Flux_CO2_16_VSLOPE[50];


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
}OP_SET;		//�����趨

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