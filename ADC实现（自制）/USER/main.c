#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "adc.h"
#include "SysTick.h"
#include "servo.h"

 u8 star_flag;
 u16 press_time;
 u32 time_cnts;
 int main(void)
 {	 
    u16 adcx;
	float temp;
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	TIM2_Int_Init(99,7199);//10Khz的计数频率，计数到100为10ms
	KEY_EXTIX_Init();
	uart_init(115200);	 	//串口初始化为115200
 	LED_Init();			     //LED端口初始化
	Servo_PWM_Init(100);
	LCD_Init();			 	
 	Adc_Init();		  		//ADC初始化
	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(40,50,200,16,24,"Just for Jump");	
	//显示提示信息
	POINT_COLOR=BLUE;//设置字体为蓝色
	LCD_ShowString(60,130,200,16,16,"Distance:  0.00mm");	      
	LCD_ShowString(60,150,200,16,16,"Press_Time: 0000ms");	       
	while(1)
	{
		adcx=Get_Adc_Average(ADC_Channel_1,10);
		press_time=21.26f*(adcx*0.01f)-47.97+300;//300为舵机响应时间                                                         
		temp=(float)adcx*0.01;
		adcx=temp;
		LCD_ShowxNum(140,130,adcx,2,16,0);
		temp*=100;
		LCD_ShowxNum(164,130,temp,2,16,0X80);
		LCD_ShowxNum(156,150,press_time,4,16,0);
		LCD_ShowxNum(140,170,time_cnts,6,16,0);
		LED0=!LED0;
		delay_ms(250);
		if(star_flag==1)
		{
			Servo_SetAngle(160);
			time_cnts=0;
			while(time_cnts<=press_time);
			Servo_SetAngle(90);
		    star_flag=0;
		}
	}
 }

