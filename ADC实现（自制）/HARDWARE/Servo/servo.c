#include "servo.h"
#define ACCURACY 10000 
//TIM3 PWM 部分初始化
//PWM 输出初始化
//arr：自动重装值
//psc：时钟预分频数
void Servo_PWM_Init(u16 Hz)
{
	u16 PrescalerValue = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器 3 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); //使能 GPIO 和 AFIO 复用功能时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //重映射 TIM3_CH2->PB5
	//设置该引脚为复用输出功能,输出 TIM3 CH2 的 PWM 脉冲波形 GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化 GPIO
	//初始化 TIM3
	PrescalerValue = (uint16_t) ( ( SystemCoreClock) / (ACCURACY*Hz) ) - 1;
	TIM_TimeBaseStructure.TIM_Period = ACCURACY; //设置在自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue; //设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化 TIMx
	//初始化 TIM3 Channel2 PWM 模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择 PWM 模式 2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); //初始化外设 TIM3 OC2
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能预装载寄存器
	TIM_Cmd(TIM3, ENABLE); //使能 TIM3
}
void Servo_SetAngle(u16 angle)
{
	u16 temp;
	temp=angle*11.11+500;
	TIM_SetCompare2(TIM3,temp);
}
