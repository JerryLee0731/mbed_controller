#include "motor.h"

static double MIN_PWM = 0.098; // 0.37
static double MIN_VEL = 10;

double enc2deg (int pulse) {
    return static_cast<double>(pulse) * 360.0f / 1200.0f;
}

double motor (double x) {
    double control = (1-MIN_PWM) * x + MIN_PWM;

    return control;
}

void deadzone (double& x, double min, double max) {
    if (x > min && x < max) {
        x = 0;
    }
}

void setMotorSpeed(double control, PwmOut& pwm_1, PwmOut& pwm_2) {
    if (control > 0) {
        pwm_1.write(0);
        if (control > 1) {
            control = 1;
        }
        pwm_2.write(motor(control));
    } // 0到1之間
    else if (control < 0) {
        pwm_2.write(0);
        if (control < -1) {
            control = -1;
        }
        pwm_1.write(motor(-control));
    }
    else {
        pwm_1.write(0);
        pwm_2.write(0);
    }
}

double bound01 (double x) {
    if (x > 1) {
        x = 1;
    }
    if (x < -1) {
        x = 0;
    }
    return x;
}

double ff (double target) {
    if (target > 0) {
        double ans = (target + 37.33) / 734.24; // from my test, recorded in excel
        return bound01(ans);
    }
    else if (target == 0) {
        return 0;
    }
    else {
        double ans = (-target + 313.19) / 1194.5;
        return -bound01(ans);
    }
}

void motorControl_ang(double target, double input, PIDController pid, PwmOut& pwm_1, PwmOut& pwm_2) {
    pid.setSetpoint(target);
    double control = pid.compute(input);
    double dz = 1e-3;
    deadzone(control, -dz, dz);
    //printf("control = %d\r\n", int(control * 1e6));
    setMotorSpeed(control, pwm_1, pwm_2);
}

void motorControl_vel(double target, double input, PIDController pid, PwmOut& pwm_1, PwmOut& pwm_2) {
    // motor control vel, add feedfoward term
    deadzone(target, -MIN_VEL, MIN_VEL);
    //printf("target_vel = %d\r\n", int(target));
    //printf("ff = %d\r\n", int(ff(target) * 1000));
    pid.setSetpoint(target);
    double control = pid.compute(input);
    control = control + ff(target);
    //printf("control = %d\r\n", int(control * 1000));
    setMotorSpeed(control, pwm_1, pwm_2);
}