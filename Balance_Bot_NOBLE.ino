#include "Arduino_BMI270_BMM150.h"
#include "KalmanFilter.h"
#include <String.h>
#include "Balbot_RGB.h"

#define SYS_ENABLE_PIN D12
#define M_BIN1 D5
#define M_BIN2 D4
#define M_AIN2 D3
#define M_AIN1 D2
KalmanFilter kalmanX(0.002, 0.000001, 0.05);
KalmanFilter kalmanY(0.002, 0.000001, 0.05);
float gyro_x, gyro_y, gyro_z;  // nRF52840 has FPU
float accel_x, accel_y, accel_z;

float motorPower = 0;

float Kp = 9.5;
float Kd = 0.16;
float Ki = 135;
float accPitch = 0;
float kalPitch = 0;
float errorAngle = 0;
float errorSum = 0;
float prevAngle = 0;
float max_errorSum = 140;
float eraseErrorMillis = 0;
float targetAngle = 0.022;
static float maxAngleError = 30;
static int motorMin = 250;
int controlloop_dt = 0;
int leftturn_flag = 0;
int rightturn_flag = 0;
float DT = 0.011; // 90Hz Control Loop
int controlloop_micros = 0;

void driveMotor(int AIN1_pwm, int AIN2_pwm, int BIN1_pwm, int BIN2_pwm){
  
  analogWrite(M_AIN1, AIN1_pwm);  // 255 (always on), 0 (always off)
  analogWrite(M_AIN2, AIN2_pwm);

  // NOTE, BIN WILL HAVE THE OPPOSITE SIGNAL AS AIN TO DRIVE IN THE SAME DIRECTION
  analogWrite(M_BIN1, BIN1_pwm);
  analogWrite(M_BIN2, BIN2_pwm);
}

void init_IMU(void){

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Angular speed in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

}

void controlloop();

void setup() {
  init_IMU();

  pinMode(M_BIN1, OUTPUT);
  pinMode(M_BIN2, OUTPUT);
  pinMode(M_AIN2, OUTPUT);
  pinMode(M_AIN1, OUTPUT);

  pinMode(SYS_ENABLE_PIN, INPUT);
  pinMode(D8, INPUT);
  pinMode(D7, INPUT);
  pinMode(D6, INPUT);
}

void loop() {
  if(digitalRead(SYS_ENABLE_PIN) == 1){

    controlloop();

    if((digitalRead(D8) == 0) && (digitalRead(D7) == 0) && (digitalRead(D6) == 0)){
      targetAngle = 0;
      leftturn_flag = 0;
      rightturn_flag = 0;
    }
    else if((digitalRead(D8) == 0) && (digitalRead(D7) == 0) && (digitalRead(D6) == 1)){
      targetAngle = 0.75;
      leftturn_flag = 0;
      rightturn_flag = 0;
    }
    else if((digitalRead(D8) == 0) && (digitalRead(D7) == 1) && (digitalRead(D6) == 0)){
      targetAngle = -0.75;
      leftturn_flag = 0;
      rightturn_flag = 0;
    }
    else if((digitalRead(D8) == 0) && (digitalRead(D7) == 1) && (digitalRead(D6) == 1)){
      targetAngle = 0;
      leftturn_flag = 1;
      rightturn_flag = 0;
    }
    else if((digitalRead(D8) == 1) && (digitalRead(D7) == 0) && (digitalRead(D6) == 0)){
      targetAngle = 0;
      leftturn_flag = 0;
      rightturn_flag = 1;
    }
    else {
      targetAngle = 0;
      leftturn_flag = 0;
      rightturn_flag = 0;
    }
  }
  else {
    driveMotor(0, 0, 0, 0);
  }
}

void controlloop(){
    controlloop_dt = micros() - controlloop_micros;
    if(controlloop_dt >= DT*1000000){
        //if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        //}
      if(1){ // IMU.accelerationAvailable()
          IMU.readAcceleration(accel_x, accel_y, accel_z);
          accPitch = -(atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0) / M_PI;
          prevAngle = kalPitch;
          kalPitch = kalmanY.update(accPitch, gyro_x);
          Serial.println(kalPitch);
          errorAngle = targetAngle - kalPitch;
          errorAngle = constrain(errorAngle, -1.0*maxAngleError, maxAngleError);
          errorSum = errorSum + errorAngle;
          errorSum = constrain(errorSum, -max_errorSum, max_errorSum);
          motorPower = Kp*(errorAngle) + Ki*(errorSum)*((float)controlloop_dt/1000000.0f) - Kd*(kalPitch-prevAngle)/((float)controlloop_dt/1000000.0f);
          motorPower = constrain(motorPower, -255, 255);
          if((motorPower > 0) && (rightturn_flag == 0) && (leftturn_flag == 0)){
            motorPower = (255-motorPower);
            motorPower = (motorPower*(float)motorMin)/255.0;
            motorPower = constrain(motorPower, 0, motorMin);
            driveMotor(255, motorPower, motorPower, 255);
          }
          else if ((motorPower < 0) && (leftturn_flag == 0) && (rightturn_flag == 0)){
            motorPower = 255 - (-1*motorPower);
            motorPower = (motorPower*(float)motorMin)/255.0;
            motorPower = constrain(motorPower, 0, motorMin);
            driveMotor(motorPower, 255, 255, motorPower);
          }
          else if (rightturn_flag){
            if (motorPower > 0){
              motorPower = (255-motorPower);
              motorPower = (motorPower*(float)motorMin)/255.0;
              motorPower = constrain(motorPower, 0, motorMin);
              driveMotor(255, motorPower*1.1, motorPower, 255);          
            }
            else {
              motorPower = 255 - (-1*motorPower);
              motorPower = (motorPower*(float)motorMin)/255.0;
              motorPower = constrain(motorPower, 0, motorMin);
              driveMotor(motorPower, 255, 255, motorPower*1.1);
            }
          }
          else if (leftturn_flag){
            if (motorPower > 0){
              motorPower = (255-motorPower);
              motorPower = (motorPower*(float)motorMin)/255.0;
              motorPower = constrain(motorPower, 0, motorMin);
              driveMotor(255, motorPower, motorPower*1.1, 255);          
            }
            else {
              motorPower = 255 - (-1*motorPower);
              motorPower = (motorPower*(float)motorMin)/255.0;
              motorPower = constrain(motorPower, 0, motorMin);
              driveMotor(motorPower*1.1, 255, 255, motorPower);
            }
          }
        controlloop_micros = micros();
      }
    } // End of Control Loop
}

