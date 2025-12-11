#include "Motor.h"
#include "Sender.h"

Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Sender Sender;

void setup() {
  Motor.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  Serial.begin(115200);
  delay(100);
}

void loop() { 
}