#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    void setTunings(double kp, double ki, double kd);
    void setSetpoint(double setpoint);
    void setSampleTime(double sampleTime);
    void setOutputLimits(double min, double max);
    double compute(double input);

private:
    double kp;
    double ki;
    double kd;
    double setpoint;
    double sampleTime;
    double outputMin;
    double outputMax;

    double prevInput;
    double integral;
    double lastTime;
};

#endif // PIDCONTROLLER_H