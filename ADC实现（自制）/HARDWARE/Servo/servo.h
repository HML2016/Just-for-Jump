#ifndef __SERVO_H
#define __SERVO_H
#include "sys.h"

void Servo_PWM_Init(u16 Hz);
void Servo_SetAngle(u16 angle);
#endif
