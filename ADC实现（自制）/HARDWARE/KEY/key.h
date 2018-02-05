#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键3(WK_UP) 


void KEY_EXTIX_Init(void);
				    
#endif
