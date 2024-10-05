#ifndef MOTOR_H
#define MOTOR_H

#include <mbed.h>
#include "pidController.h"

double enc2deg (int pulse);
double motor (double x);
void deadzone (double& x, double min, double max);
void setMotorSpeed(double control, PwmOut& pwm_1, PwmOut& pwm_2);
double bound01 (double x);
double ff (double target);
void motorControl_ang(double target, double input, PIDController pid, PwmOut& pwm_1, PwmOut& pwm_2);
void motorControl_vel(double target, double input, PIDController pid, PwmOut& pwm_1, PwmOut& pwm_2);

#endif //MOTOR_H