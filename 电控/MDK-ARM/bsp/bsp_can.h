/********************************************************************************************************
*                                DJI System Global Macros Definations
*
*                                   (c) Copyright 2017; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
********************************************************************************************************/
/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_can.h
	* @brief      receive external can device message, motor/gyroscope/module etc...
	*             get motor encoder initial offset, calculate motor speed 
	*             according to return encoder data
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   
  * @verbatim
	*   
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"

/* CAN send and receive ID */
typedef enum {

	/* CAN1 */
	CAN_GIMBAL_ANG_ID 	 = 0x100,
	CAN_GIMBAL_FLG_ID	 = 0x101,
	CAN_GIMBAL_ID		 = 0x102,
	CAN_3508_M1_ID       = 0x201,
	CAN_3508_M2_ID       = 0x202,
	CAN_3508_M3_ID       = 0x203,
	CAN_3508_M4_ID       = 0x204,
	CAN_YAW_MOTOR_ID     = 0x205,		//云台的6623 205-206
	CAN_PIT_MOTOR_ID     = 0x206, 
	CAN_TRIGGER_MOTOR_ID = 0x207,		//云台的小弹丸拨弹
	
	/* CAN2 */
	CAN_FRIC_3508_L_ID		= 0x207,
	CAN_FRIC_3508_R_ID		= 0x208,
//	CAN_NEW_ZGYRO_ID     	= 0x208,
	CAN_OLD_ZGYRO_ID     	= 0x401,
	CAN_CHASSIS_ZGYRO_ID 	= 0x402,
	
	CAN_ZGYRO_RST_ID     	= 0x404,
	CAN_CHASSIS_ALL_ID   	= 0x200,
	CAN_GIMBAL_ALL_ID    	= 0x1ff,

} can_msg_id_e;

#define FILTER_BUF_LEN 5
/* can receive motor parameter structure */
typedef struct				//can both work in 3510 & 6623 & 3508
{
  int16_t ecd;
  int16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;			//Totally pass encoder nums
  int32_t  total_angle;		//Totally pass angle in degree
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
} moto_measure_t;

typedef struct
{
	float yaw_angle;
	float yaw_omega;
}ZGyro_t;
extern ZGyro_t ZGyro;

typedef union
{
	uint8_t u8t[8];
	float ft[2];
	int16_t i16t[4];
	uint32_t u32t[2];
	int32_t i32t[2];
}data_union_t;


//extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_yaw, moto_pit;
extern moto_measure_t moto_fric_l, moto_fric_r;
extern moto_measure_t moto_trigger;
extern moto_measure_t moto_3508;
//extern moto_measure_t moto_maga_golf,moto_maga_bullet;
//extern moto_measure_t moto_l,moto_r;
//extern moto_measure_t moto_clamp,moto_rotate;
extern float          yaw_zgyro_angle;

void get_moto_measure(moto_measure_t* ptr, CAN_HandleTypeDef* hcan);
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan);
void moto_ecd_reset(moto_measure_t* ptr);
void set_200_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

void my_can_filter_init_recv_all(void);

void reset_zgyro(void);

#endif
