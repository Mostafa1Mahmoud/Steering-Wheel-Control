#include <Arduino.h>
#include "MPU9250.h"

MPU9250 mpu;
bool Direction = 0, Reset = 0;
int Counter, Slots, leftPulses = 0 , rightPulses = 0 , Resistance , Radiuse , totalLaps;
double Voltage, Current, leftSpeed = 0.0 , rightSpeed = 0.0,Time = 0.0,usedCurrent = 0.0, SOC = 0.0, TotalCharge = 0;
double accelerationX = 0.0, accelerationY = 0.0, accelerationZ = 0.0, gyroX = 0.0,gyroY = 0.0, gyroZ = 0.0;
double accelerationBiasX = 0.0, accelerationBiasY = 0.0, accelerationBiasZ = 0.0, gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
double magBiasX = 0.0, magBiasY = 0.0, magBiasZ = 0.0, magX = 0.0, magY = 0.0, magZ = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
unsigned long currentTime, lastTime , Interval = 0, socInterval = 1, socCurrentTime = 0, socLastTime = 0;
char directionPin = 13 , counterPin = 12, leftEncoderPin = 2,rightEncoderPin = 3, resetPin = 7, voltagePin = A0;

void mpuCalibrate(){
  Wire.begin();
  delay(2000);
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();
  print_calibration();
  mpu.verbose(false);
}

void calAccelerationBias(){
  accelerationBiasX = mpu.getAccBiasX() * 1000.lf / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
  accelerationBiasY = mpu.getAccBiasY() * 1000.lf / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
  accelerationBiasZ = mpu.getAccBiasZ() * 1000.lf / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
}
void calAcceleration(){
  accelerationX = getAccX();
  accelerationY = getAccY();
  accelerationZ = getAccZ();
}

void calGyroBias(){
  mpu.update_accel_gyro();
  gyroBiasX = mpu.getGyroBiasX() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
  gyroBiasY = mpu.getGyroBiasY() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
  gyroBiasZ = mpu.getGyroBiasZ() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
}

void calGyro(){
  mpu.update_accel_gyro();
  gyroX = getGyroX();
  gyroY = getGyroY();
  gyroZ = getGyroZ();
}
void calMagBias(){
  magBiasX = getMagBiasX();
  magBiasY = getMagBiasY();
  magBiasZ = getMagBiasZ();
}
void calMag(){
  update_mag();
  magX = getMagX();
  magY = getMagY();
  magZ = getMagZ();
}
void calRollPitchYaw() {
    yaw = mpu.getYaw();
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
}
void setup() {
  // put your setup code here, to run once:
  pinMode(directionPin     ,INPUT);
  pinMode(counterPin       ,INPUT);
  pinMode(leftEncoderPin   ,INPUT);
  pinMode(rightEncoderPin  ,INPUT);
  pinMode(resetPin         ,INPUT);
  pinMode(voltagePin       ,INPUT);

  // enableing Interrupt for Pin
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin)  , leftISR ,  RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin) , rightISR , RISING);
  
}

double battaryCounter(){
  socCurrentTime = millis() - socLastTime;
  if(socCurrentTime >= socInterval){
    SOC +=  Current*.001;
    socLastTime = socCurrentTime;
  }
  return ((TotalCharge - SOC)/TotalCharge) * 100;
}

void leftISR(){
    leftPulses++;
}

void rightISR(){
    rightPulses++;
}

void calcSpeedLeft(){
    double Revelution;
    currentTime = millis() - lastTime;
    if(currentTime >= Interval){
      Revelution = (double)leftPulses / Slots;
      leftPulses = 0;
      leftSpeed = Revelution*(2*PI*Radiuse) / currentTime;
      lastTime = currentTime;
    }
}

void calcSpeedRight(){
    double Revelution;
    currentTime = millis() - lastTime;
    if(currentTime >= Interval){
      Revelution = (double)rightPulses / Slots;
      rightPulses = 0;
      rightSpeed = Revelution*(2*PI*Radiuse) / currentTime;
      lastTime = currentTime;
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  // the Direction of the wheel
  Direction = digitalRead(directionPin);
  
  // Handling The reset & Laps Buttons
  if(digitalRead(resetPin) && digitalRead(counterPin)){
    totalLaps = Counter;
  }
  else if(digitalRead(resetPin) && !digitalRead(counterPin)){
    Counter = 0;
  }
  else if(!digitalRead(resetPin) && digitalRead(counterPin)){
    Counter++;
  }

  // the voltage & current come from the battery
  Voltage   = analogRead(voltagePin);
  Current   = Voltage / Resistance;

  Time = millis() - Time;
  usedCurrent = Time * Current;

  calcSpeedLeft();
  calcSpeedRight();

}
