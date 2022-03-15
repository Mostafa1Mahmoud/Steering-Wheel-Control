#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "MPU9250.h"
#include "Wire.h"

MPU9250 mpu;
File MyFile;

bool Direction = 0, Reset = 0;
int Counter, Slots = 1, leftPulses = 0 , rightPulses = 0 , Resistance , Radiuse = 1, totalLaps, pinCS = 53;
double Voltage, Current, leftSpeed = 0.0 , rightSpeed = 0.0,Time = 0.0,usedCurrent = 0.0, SOC = 0.0, TotalCharge = 0;
double accelerationX = 0.0, accelerationY = 0.0, accelerationZ = 0.0, gyroX = 0.0,gyroY = 0.0, gyroZ = 0.0;
double accelerationBiasX = 0.0, accelerationBiasY = 0.0, accelerationBiasZ = 0.0, gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
double magBiasX = 0.0, magBiasY = 0.0, magBiasZ = 0.0, magX = 0.0, magY = 0.0, magZ = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
double sdTime = 0.0, sdLastTime = 0.0, sdMarginTime = 25;
unsigned long leftCurrentTime = 0, leftLastTime = 0,rightCurrentTime = 0, rightLastTime = 0 , Interval = 25, socInterval = 1, socCurrentTime = 0, socLastTime = 0;
char directionPin = 13 , counterPin = 12, leftEncoderPin = 2,rightEncoderPin = 3, resetPin = 7, voltagePin = A0;

void mpuCalibrate(){
  Wire.begin();
  delay(2000);
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
//  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();
//  Print_calibration();
//  mpu.verbose(false);
  
}

void calAccelerationBias(){
  mpu.update();
  accelerationBiasX = mpu.getAccBiasX() * 1000.f / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
  accelerationBiasY = mpu.getAccBiasY() * 1000.f / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
  accelerationBiasZ = mpu.getAccBiasZ() * 1000.f / (double)MPU9250::CALIB_ACCEL_SENSITIVITY;
}
void calAcceleration(){
  mpu.update();
  accelerationX = mpu.getAccX();
  accelerationY = mpu.getAccY();
  accelerationZ = mpu.getAccZ();
}

void calGyroBias(){
  mpu.update_accel_gyro();
  gyroBiasX = mpu.getGyroBiasX() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
  gyroBiasY = mpu.getGyroBiasY() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
  gyroBiasZ = mpu.getGyroBiasZ() / (double)MPU9250::CALIB_GYRO_SENSITIVITY;
}

void calGyro(){
  mpu.update_accel_gyro();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
}
void calMagBias(){
  magBiasX = mpu.getMagBiasX();
  magBiasY = mpu.getMagBiasY();
  magBiasZ = mpu.getMagBiasZ();
}
void calMag(){
  mpu.update_mag();
  magX = mpu.getMagX();
  magY = mpu.getMagY();
  magZ = mpu.getMagZ();
}
void calRollPitchYaw() {
    
    yaw = mpu.getYaw();
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  mpu.setup(0x68);  // change to your own address depends on AD0 on MPU 9250 if(AD0 == Low)I2C address is 0x68 else I2C address is 0x69
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();
  mpu.verbose(false);
  Serial.println("after Mag Calibration");
  SD.begin();
  Serial.println("after initializing SD Card");
  // put your setup code here, to run once:
  pinMode(directionPin     ,INPUT);
  pinMode(counterPin       ,INPUT);
  pinMode(leftEncoderPin   ,INPUT);
  pinMode(rightEncoderPin  ,INPUT);
  pinMode(resetPin         ,INPUT);
  pinMode(voltagePin       ,INPUT);
  pinMode(pinCS            ,OUTPUT);
  // enableing Interrupt for Pin
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin)  , leftISR ,  RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin) , rightISR , RISING);
  //mpuCalibrate();
  calGyroBias();
  calAccelerationBias();
  calMagBias();
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
    leftCurrentTime = millis() - leftLastTime;
    if(leftCurrentTime >= Interval){
      Revelution = (double)leftPulses / Slots;
      leftPulses = 0;
      leftSpeed = Revelution*(2*PI*Radiuse) / leftCurrentTime;
      leftLastTime = millis();
    }
}

void calcSpeedRight(){
    double Revelution;
    rightCurrentTime = millis() - rightLastTime;
    if(rightCurrentTime >= Interval){
      Revelution = (double)rightPulses / Slots;
      rightPulses = 0;
      rightSpeed = Revelution*(2*PI*Radiuse) / rightCurrentTime;
      rightLastTime = millis();
    }
}

void loop() {
  mpu.update();
  calAcceleration();
  calGyro();
  calMag();
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

//  // the voltage & current come from the battery
//  Voltage   = analogRead(voltagePin);
//  Current   = Voltage / Resistance;
//
//  Time = millis() - Time;
//  usedCurrent = Time * Current;

  calcSpeedLeft();
  calcSpeedRight();

  /*sdTime = millis() - sdLastTime;
  if(sdTime > sdMarginTime){
    sdLastTime = millis();
    MyFile = SD.open("Data.txt", FILE_WRITE);  
    MyFile.println(ٍString(rightSpeed)+","+String(leftSpeed)+","+String(accelerationX)+","+String(accelerationY)+","+String(accelerationZ)+","+String(gyroX - gyroBiasX)+","+String(gyroY - gyroBiasY)+","+String(gyroZ - gyroBiasZ)+","+String(magX - magBiasX)+","+String(magY - magBiasY)+","+String(magZ - magBiasZ)+","+String(mpu.getYaw()));
    MyFile.close();
  }*/
//  Serial.println("rightSpeed: " + String(rightSpeed));
//  Serial.println("leftSpeed: " + String(leftSpeed)); 
  /*currentTime = millis() - lastTime;
  if(currentTime > Interval){
      lastTime = millis();
      currentTime = 0;
            
      //Serial.println("accelerationX: " + String(accelerationX));
      //Serial.println("accelerationY: " + String(accelerationY));
      //Serial.println("accelerationZ: " + String(accelerationZ));
      
      //Serial.println("accelerationBiasX: " + String(accelerationBiasX));
      //Serial.println("accelerationBiasY: " + String(accelerationBiasY));
      //Serial.println("accelerationBiasZ: " + String(accelerationBiasZ));
      
      //Serial.println("Total gyroX: " + String(gyroX - gyroBiasX));
      //Serial.println("Total gyroY: " + String(gyroY - gyroBiasY));
      //Serial.println("Total gyroZ: " + String(gyroZ - gyroBiasZ));
      
      //Serial.println("gyroX: " + String(gyroX));
      //Serial.println("gyroY: " + String(gyroY));
      //Serial.println("gyroZ: " + String(gyroZ));
      
      //Serial.println("gyroBiasX: " + String(gyroBiasX));
      //Serial.println("gyroBiasX: " + String(gyroBiasY));
      //Serial.println("gyroBiasZ: " + String(gyroBiasZ));
      
      //Serial.println("Total magX: "+ String(magX - magBiasX));
      //Serial.println("Total magY: "+ String(magY - magBiasY));
      //Serial.println("Total magZ: "+ String(magZ - magBiasZ));
      
      //Serial.println("magX: "+ String(magX));
      //Serial.println("magY: "+ String(magY));
      //Serial.println("magZ: "+ String(magZ));
      
      //Serial.println("magBiasX: "+ String(magBiasX));
      //Serial.println("magBiasY: "+ String(magBiasY));
      //Serial.println("magBiasZ: "+ String(magBiasZ));
      
      //Serial.println("Yaw: " + String(mpu.getYaw()));
    }*/
}
