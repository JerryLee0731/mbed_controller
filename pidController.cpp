#include "pidController.h"
#include "mbed.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0), sampleTime(1.0), outputMin(0), outputMax(1),
      prevInput(0), integral(0), lastTime(0) {}

void PIDController::setTunings(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setSampleTime(double sampleTime) {
    this->sampleTime = sampleTime;
}

void PIDController::setOutputLimits(double min, double max) {
    if (min >= max) return;
    outputMin = min;
    outputMax = max;
}

double PIDController::compute(double input) {
    double now = us_ticker_read() / 1000000.0; // Convert to seconds
    double timeChange = (now - lastTime);
    
    if (timeChange >= sampleTime) {
        double error = setpoint - input;
        //printf("error = %d\r\n", int(error));
        integral += (ki * error * timeChange);
        if (integral > outputMax) integral = outputMax;
        else if (integral < outputMin) integral = outputMin;

        double dInput = (input - prevInput) / timeChange;

        double output = kp * error + integral - kd * dInput;
        //printf("%d\r\n", int(output * 1000));
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;
        //printf("output_after = %d\r\n", int(output * 1000));

        prevInput = input;
        lastTime = now;

        return output;
    }
    return 0;
}