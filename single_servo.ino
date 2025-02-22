#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

#define SERVOMIN  95  // Adjust if needed
#define SERVOMAX  525  // Adjust if needed

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685...");
  
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency

  delay(1000);
}

void loop() {
  Serial.println("Moving servo to 0 degrees...");
  pwm.setPWM(0, 0, SERVOMIN);
  delay(1000);
  
  // Serial.println("Moving servo to 90 degrees...");
  // pwm.setPWM(0, 0, (SERVOMIN + SERVOMAX) / 2);
  // delay(1000);
  
  // Serial.println("Moving servo to 180 degrees...");
  // pwm.setPWM(0, 0, SERVOMAX);
  // delay(1000);
}
