#include "Sender.h"
#include "Wire.h"
#include "Motor.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"

MPU6050 MPU6050;
Mpu6050 Mpu6050;
Sender Sender;
KalmanFilter kalmanfilter;
Motor Motor;

void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
}

static void Timer2::interrupt()
{ 
  if (Sender.t0 == 0) {
    Sender.t0 = millis();
  }
//  Sender.t0 = millis();
//  Serial.print("T:"); Serial.println(Sender.t0 - Sender.lasT);
  sei();//enable the global interrupt
  Sender.getEncoderSpeed();
  Mpu6050.DataProcessing();
  Sender.sendData();
  Sender.readData();
  Sender.controlMotors();
//  Serial.print("T:"); Serial.println(Sender.responseTime);
//  Sender.lasT = Sender.t0;
}

Sender::Sender(){ }

void Sender::sendData() 
{
  float angle = kalmanfilter.angle;
  float Gyro_x = kalmanfilter.Gyro_x;
  float Gyro_z = kalmanfilter.Gyro_z;

  int e_l_p_n_speed = encoder_left_pulse_num_speed;
  int e_r_p_n_speed = encoder_right_pulse_num_speed;

  encoder_left_pulse_num_speed = 0;
  encoder_right_pulse_num_speed = 0;

  Serial.print("A:"); Serial.print(angle); Serial.print(","); Serial.print(i);
  Serial.print(" G:"); Serial.print(Gyro_x); Serial.print(","); Serial.print(Gyro_z);
  Serial.print(" S:"); Serial.print(e_l_p_n_speed); Serial.print(","); Serial.println(e_r_p_n_speed);
  i++;

  if (i == 10000) {
    last_i = -1;
    i = 0;
  }
}

void Sender::readData()
{
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    
    int i_index = command.indexOf("i:");
    int new_i = (i_index != -1) ? command.substring(i_index + 2).toInt() : -1;


    if (new_i >= last_i) {
      last_i = new_i;

      left_pwm_index = command.indexOf("L:");
      right_pwm_index = command.indexOf("R:");

      if (left_pwm_index != -1 && right_pwm_index != -1) {
        left_pwm = command.substring(left_pwm_index + 2).toDouble();
        right_pwm = command.substring(right_pwm_index + 2).toDouble();
      }

      t1 = millis();
      responseTime =  t1 - t0;
      t0 = millis();
    }

  }
}

void Sender::controlMotors() {
  while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
  { 
    Mpu6050.DataProcessing();
    Motor.Stop();
  }

  (left_pwm < 0) ? (Motor.Control(AIN1,1,PWMA_LEFT,-left_pwm)) : (Motor.Control(AIN1,0,PWMA_LEFT,left_pwm));
  (right_pwm < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-right_pwm)) : (Motor.Control(BIN1,0,PWMB_RIGHT,right_pwm));
}

void Sender::getEncoderSpeed() {
  encoder_left_pulse_num_speed += left_pwm < 0 ? (-Motor::encoder_count_left_a) : Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += right_pwm < 0 ? (-Motor::encoder_count_right_a) : Motor::encoder_count_right_a;

  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}

void Mpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
}

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
}
