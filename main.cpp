#include "mbed.h"
#include <cmath>
#include "QEI.h"
#include "pidController.h"
#include "motor.h"

double ang_f, ang_r;
double vel_f, vel_r;

double target_ang_f, target_ang_r;
double target_vel_f, target_vel_r;

double lastTime = 0;
double lastTime_ser = 0;

int enc_vel_f_prev = 0;
int enc_vel_r_prev = 0;

PwmOut pwm_vel_f1(D4); // A0, 反轉
PwmOut pwm_vel_f2(D5); // A1, 正轉 逆時針 enc正

PwmOut pwm_ang_f1(A0); // A0, 反轉
PwmOut pwm_ang_f2(A1); // A1, 正轉 逆時針 enc正

QEI encoder_vel_f(D11, D12, NC, 16); // D11 D12
QEI encoder_ang_f(D2, D3, NC, 16); // D2 D3
/*
QEI encoder_vel_r(D8, D9, NC, 16);
QEI encoder_ang_r(D10, D11, NC, 16);
*/
double kp_ang = 1e-3;
double ki_ang = 0;
double kd_ang = 0;
PIDController pid_angle(kp_ang, ki_ang, kd_ang);

double kp_vel = 1e-3;
double ki_vel = 0;
double kd_vel = 0;
PIDController pid_vel(kp_vel, ki_vel, kd_vel);

//AnalogIn a2(A2);

// uart to PC
char read_buf[512]; // must be larger tan length of string in the message !!!
char write_buf[512];

// pc <-> stm32
BufferedSerial serial_port(USBTX, USBRX, 115200);  // (tx,rx,baudrate)

// main() runs in its own thread in the OS
int main()
{
    // 初始化motor pwm週期 10000Hz
    pwm_ang_f1.period(1.0 / 10000);
    pwm_ang_f2.period(1.0 / 10000);

    pwm_vel_f1.period(1.0 / 10000);
    pwm_vel_f2.period(1.0 / 10000);

    pid_angle.setOutputLimits(-0.8, 0.8); // 设置输出范围
    pid_angle.setSampleTime(0.01); // 设置采样时间

    pid_vel.setOutputLimits(-0.8, 0.8);
    pid_vel.setSampleTime(0.01);
    
    // test MINPWM
    /*
    double minPWM = 0.00;
    while (encoder_ang_f.getPulses() == 0) {
        pwm_ang_f1.write(minPWM);
        minPWM = minPWM + 1e-3;
        ThisThread::sleep_for(10ms);
        printf("%d\r\n", int(minPWM * 1e3));
    }
    printf("minPWM = %d\r\n", int(minPWM * 1e3)); // 0.098
    */

    // 主迴圈
    while (true) {

        double now = us_ticker_read() / 1000000.0; // Convert to seconds
        double timeChange = (now - lastTime);
        lastTime = now;

        double now_ser = us_ticker_read() / 1000000.0; // Convert to seconds
        double timeChange_ser = (now_ser - lastTime_ser);

        //double voltage = a2.read();  // 读取模拟信号，返回值范围在 0 到 1 之间
        //printf("voltage = %d\r\n", int(voltage * 1000));

        // uart from pc
        if (timeChange_ser > 0.1 && serial_port.readable()) {
            memset(read_buf, 0, sizeof(read_buf)); //清空緩存區
            size_t n = serial_port.read(read_buf, sizeof(read_buf));           
            sscanf(read_buf, "%lf %lf %lf %lf\r\n", &target_ang_f, &target_vel_f, &target_ang_r, &target_vel_r); // 從read_buf讀出double
            lastTime_ser = now_ser;
        }
        // end uart from pc

        // encoder
        int enc_ang_f = encoder_ang_f.getPulses(); // getRev 16: Pulse Per Rev // 1200 per rev
        int enc_vel_f = encoder_vel_f.getPulses(); 

        int enc_ang_r = 0;
        int enc_vel_r = 0; 

        ang_f = enc2deg(enc_ang_f);
        ang_r = enc2deg(enc_ang_r);

        vel_f = enc2deg( (enc_vel_f - enc_vel_f_prev) / (timeChange) );
        vel_r = enc2deg( (enc_vel_r - enc_vel_r_prev) / (timeChange) );

        enc_vel_f_prev = enc_vel_f;
        enc_vel_r_prev = enc_vel_r;
        // encoder end
        
        /*
        // test pwm2vel
        pwm_vel_f1.write(0.1);
        printf("%d\r\n", int(vel_f * 1e3));
        */

        // motor
        //motorControl_ang(target_ang_f, ang_f, pid_angle, pwm_ang_f1, pwm_ang_f2);
        motorControl_vel(target_vel_f, vel_f, pid_vel, pwm_vel_f1, pwm_vel_f2);
        // motor end
        // uart to pc
        
        if (serial_port.writable()) {
            memset(write_buf, 0, sizeof(write_buf)); //清空緩存區
            std::snprintf(write_buf, sizeof(write_buf), "%d %d %d %d %d %d %d %d\r\n",
                int(target_ang_f*1e3), 
                int(target_vel_f*1e3), 
                int(target_ang_r*1e3), 
                int(target_vel_r*1e3), 
                int(ang_f*1e3), 
                int(vel_f*1e3), 
                int(ang_r*1e3), 
                int(vel_r*1e3)
                ); // 存入write_buf
            
            // 使用 strlen(write_buf) 确定要发送的字节数
            serial_port.write(write_buf, strlen(write_buf));
        }
        
        //send_doubles_as_string(10.00, 20.00, 30.00);
        
        ThisThread::sleep_for(10ms);
    }
}
