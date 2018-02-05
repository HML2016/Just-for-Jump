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
  * @file       bsp_can.c
	* @brief      receive external can device message, motor_esc/gyroscope/module etc...
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

#include "bsp_can.h"
#include "bsp_uart.h"
#include "string.h"



/* on the gimbal(use the yaw to make a chassis close loop) */
//moto_measure_t moto_pit;
//moto_measure_t moto_yaw;
//moto_measure_t moto_trigger;
//moto_measure_t moto_chassis[4];
moto_measure_t moto_3508;
/* lift up the whole top shelf */
//moto_measure_t moto_fric_l;
//moto_measure_t moto_fric_r;
/* on the mega */
//moto_measure_t moto_maga_golf;
//moto_measure_t moto_maga_bullet;
/* on the claw */
//moto_measure_t moto_clamp;
//moto_measure_t moto_rotate;
moto_measure_t moto_test;

//float yaw_zgyro_angle;
//float yaw_zgyro_omega;
ZGyro_t ZGyro;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	if(_hcan == &hcan1)
	{
		switch (_hcan->pRxMsg->StdId)
		{
			
			
			case CAN_3508_M1_ID:
//			case CAN_3508_M2_ID:
//			case CAN_3508_M3_ID:
//			case CAN_3508_M4_ID:
			{
//				static uint8_t i;
//				i = _hcan->pRxMsg->StdId - CAN_3508_M1_ID;

				moto_3508.msg_cnt++ <= 50 ? get_moto_offset(&moto_3508, _hcan) : get_moto_measure(&moto_3508, _hcan);
//				UpdateCheckThread(ChassisMotor1CheckThread + i);
			}
			break;

			
			default:
			{
			}
			break;
		}
	}	//end of can1
	else if(_hcan == &hcan2)
	{
		switch (_hcan->pRxMsg->StdId)
		{
			
			default:
				
				break;
		}
	}
	//
	__HAL_CAN_ENABLE_IT(_hcan,CAN_IT_FMP0);
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
//	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}



/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after get_moto_offset() function
  */
void get_moto_measure(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_ecd      = ptr->ecd;
  ptr->ecd           = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
  ptr->speed_rpm     = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
  ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
	
  if (ptr->ecd - ptr->last_ecd > 6000)
  {
    ptr->round_cnt--;
    //ptr->ecd_raw_rate = ptr->angle - ptr->last_angle - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -6000)
  {
    ptr->round_cnt++;
    //ptr->ecd_raw_rate = ptr->angle - ptr->last_angle + 8192;
  }
  else
  {
    //ptr->ecd_raw_rate = ptr->angle - ptr->last_angle;
  }
  
  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
	  //total angle/degree
		ptr->total_angle = ptr->total_ecd * 360 / 8192;
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
    ptr->ecd        = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}
/**
	* @brief reset the total encoder value ,including the offset and roundcnt
	*/
void moto_ecd_reset(moto_measure_t* ptr)
{
	ptr->offset_ecd = ptr->ecd;
	ptr->round_cnt = 0;
}

/**
	* @brief	give current for 0x201-0x204 moto of robomaster
	*/
void set_200_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    hcan->pTxMsg->StdId   = 0x200;
    hcan->pTxMsg->IDE     = CAN_ID_STD;
    hcan->pTxMsg->RTR     = CAN_RTR_DATA;
    hcan->pTxMsg->DLC     = 0x08;
    hcan->pTxMsg->Data[0] = iq1 >> 8;
    hcan->pTxMsg->Data[1] = iq1;
    hcan->pTxMsg->Data[2] = iq2 >> 8;
    hcan->pTxMsg->Data[3] = iq2;
    hcan->pTxMsg->Data[4] = iq3 >> 8;
    hcan->pTxMsg->Data[5] = iq3;
    hcan->pTxMsg->Data[6] = iq4 >> 8;
    hcan->pTxMsg->Data[7] = iq4;
    HAL_CAN_Transmit(hcan, 15);
}

/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void my_can_filter_init_recv_all(void)
{
  //can1 &can2 use same filter config
  CAN_FilterConfTypeDef  can_filter;
  static CanTxMsgTypeDef Tx1Message;
  static CanRxMsgTypeDef Rx1Message;
  static CanTxMsgTypeDef Tx2Message;
  static CanRxMsgTypeDef Rx2Message;

  can_filter.FilterNumber         = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.BankNumber           = 14; //can1(0-13)and can2(14-27)each get half filter
  can_filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);

  //filter config for can2
  //can1(0-13),can2(14-27)
  can_filter.FilterNumber = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
  hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;

}
/**
  * @brief     reset single axis gyroscope 
  * @attention gyro reset at least wait 2s  
  */
void reset_zgyro(void)
{
    while (hcan2.State == HAL_CAN_STATE_BUSY_TX);
    hcan2.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
    hcan2.pTxMsg->IDE     = CAN_ID_STD;
    hcan2.pTxMsg->RTR     = CAN_RTR_DATA;
    hcan2.pTxMsg->DLC     = 0x08;
    hcan2.pTxMsg->Data[0] = 0;
    hcan2.pTxMsg->Data[1] = 1;
    hcan2.pTxMsg->Data[2] = 2;
    hcan2.pTxMsg->Data[3] = 3;
    hcan2.pTxMsg->Data[4] = 4;
    hcan2.pTxMsg->Data[5] = 5;
    hcan2.pTxMsg->Data[6] = 6;
    hcan2.pTxMsg->Data[7] = 7;
    HAL_CAN_Transmit(&hcan2, 100);       
}
