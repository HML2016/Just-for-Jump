/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       pid.h
	* @brief      pid parameter initialization, position and delta pid calculate
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo        
  * @verbatim
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#ifndef __pid_H__
#define __pid_H__

#include "stm32f4xx_hal.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
typedef struct pid_t
{
	float p;
	float i;
	float d;

	float set;
	float get;
	float err[3];

	float pout; 
	float iout; 
	float dout; 
  float out;

	float input_max_err;    //input max err;
	float output_deadband;  //output deadband; 
	
	uint32_t pid_mode;
	uint32_t max_out;
	uint32_t integral_limit;

	void (*f_param_init)(struct pid_t *pid, 
											 uint32_t      pid_mode,
											 uint32_t      max_output,
											 uint32_t      inte_limit,
											 float         p,
											 float         i,
											 float         d);
	void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;

#define PID_PARAM_DEFAULT \
{\
  0,\
  0,\
  0,\
  0,\
  0,\
  {0,0,0},\
  0,\
  0,\
  0,\
  0,\
  0,\
  0,\
}\

typedef struct
{
	float p;
	float i;
	float d;

	float set;
	float get;
	float err[3]; //error

	float pout; 
	float iout; 
	float dout; 
	float out;

	float input_max_err;    //input max err;
	float output_deadband;  //output deadband; 

	float p_far;
	float p_near;
	float grade_range;
	
	uint32_t pid_mode;
	uint32_t max_out;
	uint32_t integral_limit;

	void (*f_param_init)(struct pid_t *pid, 
											 uint32_t      pid_mode,
											 uint32_t      max_output,
											 uint32_t      inte_limit,
											 float         p,
											 float         i,
											 float         d);
	void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} grade_pid_t;

void PID_struct_init(
	  pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float get, float set);
float pid_calc_grade(grade_pid_t *pid, float get, float set);

extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_speed;
extern pid_t pid_yaw_speed;
//extern pid_t pid_lift_l;
//extern pid_t pid_lift_r;
//extern pid_t pid_spd[4];
extern pid_t pid_3508;
extern pid_t pid_3508_speed;
extern pid_t pid_fric_l;
extern pid_t pid_fric_r;
		
extern pid_t pid_trigger;
extern pid_t pid_trigger_speed;

//extern pid_t pid_chassis_angle;
//extern pid_t pid_golf;
//extern pid_t pid_golf_speed;
//extern pid_t pid_bullet;
//extern pid_t pid_bullet_speed;
extern pid_t pid_imu_tmp;
//extern pid_t pid_rfid_arm;
//extern pid_t pid_test_speed;

#endif
