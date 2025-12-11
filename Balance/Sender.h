#ifndef _SENDER_h
#define _SENDER_h

#include "MsTimer2.h"
#include "KalmanFilter.h"

class Sender
{
  public:
          Sender();
          void sendData();
          void readData();
          void controlMotors();
          void getEncoderSpeed();

          unsigned int t0 = 0;
          unsigned int t1;
          unsigned int lasT;
          unsigned int responseTime;

          int i = 0;
          int last_i = -1;

          double left_pwm;
          double right_pwm;
          
          int encoder_left_pulse_num_speed;
          int encoder_right_pulse_num_speed;

          String command;

          int left_pwm_index;
          int right_pwm_index;

   private:
   #define ANGLE_MIN -20
   #define ANGLE_MAX 20
   #define EXCESSIVE_ANGLE_TILT (kalmanfilter.angle < ANGLE_MIN || ANGLE_MAX < kalmanfilter.angle)
   #define PICKED_UP (kalmanfilter.angle6 < -10 || 22 < kalmanfilter.angle6)
};

class Timer2
{
  public:
          void init(int time);
          static void interrupt();
  private:       
          #define TIMER 5
};

class Mpu6050
{
  public:
          void init();
          void DataProcessing();
          Mpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
};


#endif