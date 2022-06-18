#pragma once
//DRIVING MOTOR PINS
#define LH_D1 9
#define LH_D2 A1
#define LH_ENA 3
#define LH_ENB 11
#define RH_D1 12
#define RH_D2 A0     
#define RH_ENA 4
#define RH_ENB 2

//GENERAL CONSTANT
#define CMD_VEL_TIMEOUT 500
#define PI 3.14159265359
#define WHEEL_DIAMETER 0.18
#define WHEEL_RADIUS 0.09
#define MAX_RPM 430
#define GEAR_REDUCTION 14
#define TICKS_PER_METER 795
#define WHEEL_SEPARATION 0.66
#define DISABLE_PWM 0 // disable motor driver
#define MIN_PWM 0 // 10% of 255 pwm = 26 pwm = 0 rpm
#define MAX_PWM 255 // 90% of 255 pwm = 229 pwm = 1000 rpm
#define STRAIGHT_PWM 75
#define TURN_PWM 65
