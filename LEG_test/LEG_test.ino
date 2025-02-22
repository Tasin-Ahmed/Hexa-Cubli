#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Math.h>
#include <Ramp.h>

#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTL(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_PRINTL(x)
  #define DEBUG_PRINT(x)
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define J1_CHANNEL 0
#define J2_CHANNEL 1
#define J3_CHANNEL 2

#define SERVO_MIN 95  // PWM value for 0 degrees
#define SERVO_MAX 525  // PWM value for 180 degrees

const double J2L = 70.0; // length in mm
const double J3L = 150.5;
const double Y_Rest = 50.0;
const double Z_Rest = -120.0;
// const double J3_LegAngle = 15.4;
const double J3_LegAngle = 18.0;


// double J1Act = 0.0, J2Act = 0.0, J3Act = 40.0;
double J1Act = 0.0, J2Act = 0.0, J3Act = 0.0;
rampDouble J1Tar = 0.0, J2Tar = 0.0, J3Tar = 0.0;

const double pattern[][4] = {
  {  0.0,  0.0,  0.0, 1000 },
  { 0.0,  40.0,  0.0, 800 },
  // { 30.0,  40.0, 68.0, 800 },
  // { 60.0,  40.0, 78.0, 1000 },
  // { 60.0,  60.0,  56.0, 800 },
  // {  0.0,  50.0,  83.0, 1000 }
};

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  UpdatePosition(0, 0, 0);
  delay(2000);
}

void loop() {
  for (int i = 0; i < sizeof(pattern) / sizeof(pattern[0]); i++) {
    double X = pattern[i][0];
    double Y = pattern[i][1];
    double Z = pattern[i][2];
    int duration = pattern[i][3];
    
    CartesianMove(X, Y, Z);
    delay(duration);
  }
}

void CartesianMove(double X, double Y, double Z) {
  Y += Y_Rest;
  Z += Z_Rest;

  double J1 = atan(X / Y) * (180 / PI);
  double H = sqrt((Y * Y) + (X * X));
  double L = sqrt((H * H) + (Z * Z));
  double J3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
  double A = atan(Z / H) * (180 / PI);
  double J2 = (B + A);
  Serial.println("J1:");
  Serial.println(J1);
  Serial.println("J2:");
  Serial.println(J2);
  Serial.println("J3:");
  Serial.println(J3);
  UpdatePosition(J1, J2, J3);
}

void UpdatePosition(double J1, double J2, double J3) {
  pwm.setPWM(J1_CHANNEL, 0, angleToPWM(J1));
  pwm.setPWM(J2_CHANNEL, 0, angleToPWM(J2));
  pwm.setPWM(J3_CHANNEL, 0, angleToPWM(J3));
  
  // DEBUG_PRINT("Moving to Angles: J1="); DEBUG_PRINT(J1);
  // DEBUG_PRINT(", J2="); DEBUG_PRINT(J2);
  // DEBUG_PRINT(", J3="); DEBUG_PRINTL(J3);
}

int angleToPWM(double angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}
