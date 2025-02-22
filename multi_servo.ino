#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Default I2C address

// Define pulse range for servos
#define SERVOMIN  95  // 0 degrees
#define SERVOMAX  525  // 180 degrees

// Define servo channels
int servos[] = {0,1, 2 };  // Servo channels on PCA9685
int numServos = sizeof(servos) / sizeof(servos[0]);  // Number of servos

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685...");

  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz frequency for servos

  delay(1000);
}

void loop() {
  // Move all servos from 0° to 180°
  // Serial.println("Moving servos to 180 degrees...");
  // for (int i = 0; i < numServos; i++) {
  //   pwm.setPWM(servos[i], 0, SERVOMAX);
  //   }
  // delay(1000);
  
  // delay(1000);
    // pwm.setPWM(servos[i], 0, (SERVOMIN + SERVOMAX) / 2);
  // delay(1000);

    // pwm.setPWM(servos[i], 0, SERVOMIN);



  // Move all servos to 90° position
  // Serial.println("Moving servos to 90 degrees...");
  // for (int i = 0; i < numServos; i++) {
  //   pwm.setPWM(servos[i], 0, (SERVOMIN + SERVOMAX) / 2);
  // }
  // delay(1000);

  // Move all servos from 180° back to 0°
  Serial.println("Moving servos to 0 degrees...");
  for (int i = 0; i < numServos; i++) {
    pwm.setPWM(servos[i], 0, SERVOMIN);
  }
  delay(1000);
}
