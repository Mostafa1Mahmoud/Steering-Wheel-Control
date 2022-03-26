#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <MPU9250.h>
#include "Wire.h"
#include <Nextion.h>

MPU9250 mpu;

File MyFile;

unsigned long previousMillis1 = 0;
const long misec = 1;
int x=0,y=0,z=0;

bool Direction = 0, Reset = 0,Right = 1, Left = 1, lastRight = 0, lastLeft = 0;
int Counter, Slots = 12, leftPulses = 0 , rightPulses = 0 , Resistance , totalLaps, pinCS = 10, FRPulses = 0, FLPulses = 0,RR = 1, RL = 1,FR = 1, FL = 1, FLLast = 1, FRLast = 1,RLLast = 1, RRLast = 1;
double Voltage, Current, leftSpeed = 0.0 , rightSpeed = 0.0,Time = 0.0,usedCurrent = 0.0, SOC = 0.0, TotalCharge = 0,FLSpeed = 0.0, FRSpeed = 0.0;
double accelerationX = 0.0, accelerationY = 0.0, accelerationZ = 0.0, gyroX = 0.0,gyroY = 0.0, gyroZ = 0.0;
double accelerationBiasX = 0.0, accelerationBiasY = 0.0, accelerationBiasZ = 0.0, gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
double magBiasX = 0.0, magBiasY = 0.0, magBiasZ = 0.0, magX = 0.0, magY = 0.0, magZ = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
double sdTime = 0.0, sdLastTime = 0.0, sdMarginTime = 500;
unsigned long leftCurrentTime = 0, leftLastTime = 0,rightCurrentTime = 0, rightLastTime = 0 , Interval = 1000, FInterval = 1000, socInterval = 1, socCurrentTime = 0, socLastTime = 0;
char directionPin = 27 , counterPin = 12, leftEncoderPin = 3,rightEncoderPin = 2, resetPin = 7, voltagePin = A0;
unsigned long mpuCurrentTime = 0, mpuLastTime = 0, mpuInterval = 200,FRCurrentTime = 0.0, FLCurrentTime = 0.0, FRLastTime = 0.0, FLLastTime = 0.0;
double rpmRR = 0.0, rpmRL = 0.0, rpmFR = 0.0, rpmFL = 0.0, Radiuse = .305;
int leftSignal = A0, rightSignal = A1;
  
NexNumber n6 = NexNumber(0,5,"n6");
NexNumber n7 = NexNumber(0,6,"n7");
NexNumber n8 = NexNumber(0,7,"n8");

NexNumber n11 = NexNumber(0,11,"n11");
NexNumber n12 = NexNumber(0,12,"n12");
NexNumber n13 = NexNumber(0,13,"n13");

NexVariable kmh = NexVariable(0,22,"kmh");
NexVariable rbm = NexVariable(0,23,"rbm");

NexVariable sv = NexVariable(0,32,"sv");

NexVariable FRX = NexVariable(0,34,"FRX");
NexVariable FLX = NexVariable(0,37,"FLX");
NexVariable RRX = NexVariable(0,35,"RRX");
NexVariable RLX = NexVariable(0,36,"RLX");

NexVariable left = NexVariable(0,28,"left");
NexVariable right = NexVariable(0,29,"right");

NexText t0 = NexText(0,4,"t0");



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

void timex(void){
  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= misec){
    previousMillis1 = currentMillis1;
    n6.setValue(++z);
    if(z>=10){
      z=0;
      n7.setValue(++x);
      if(x>=59){
      x=0;
      n8.setValue(++y);
    }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  nexInit();
  delay(1000);
  mpu.setup(0x68);  // change to your own address depends on AD0 on MPU 9250 if(AD0 == Low)I2C address is 0x68 else I2C address is 0x69
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(1000);
  mpu.calibrateAccelGyro();
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(1000);
  mpu.calibrateMag();
  mpu.verbose(false);
  Serial.println("after Mag Calibration");
  SD.begin();
  Serial.println("after initializing SD Card");
  // put your setup code here, to run once:
  pinMode(directionPin     ,INPUT_PULLUP);
  pinMode(leftEncoderPin   ,INPUT);
  pinMode(rightEncoderPin  ,INPUT);
  pinMode(leftSignal       ,INPUT_PULLUP);
  pinMode(rightSignal      ,INPUT_PULLUP);
  pinMode(pinCS            ,OUTPUT);
  bitClear(DDRE,6);
  bitClear(DDRE,7);
  bitSet(PORTE,6);
  bitSet(PORTE,7);
  //mpuCalibrate();
  calGyroBias();
  calAccelerationBias();
  calMagBias();
}

void loop() {
  mpu.update();
  calAcceleration();
  calGyro();
  calMag();
  
//  // Handling The reset & Laps Buttons
//  if(digitalRead(resetPin) && digitalRead(counterPin)){
//    totalLaps = Counter;
//  }
//  else if(digitalRead(resetPin) && !digitalRead(counterPin)){
//    Counter = 0;
//  }
//  else if(!digitalRead(resetPin) && digitalRead(counterPin)){
//    Counter++;
//  }

 
  //calcSpeedLeft();
  RL = digitalRead(leftEncoderPin);  //FR = bitRead(PINE, 6);    
  Serial.println("RL: "+String(RL));
  if(RL == 0 && RLLast == 1) {
    leftPulses++;
    RLLast = RL;
  }else {
    RLLast = RL;  
  }
  leftCurrentTime = millis() - leftLastTime;
    if(leftCurrentTime >= Interval){
      double Revelution = (double)leftPulses / Slots;
      leftSpeed = Revelution*(2*PI*Radiuse) / (leftCurrentTime/1000);
      rpmRL = Revelution*60/(leftCurrentTime/1000);
      leftLastTime = millis();
      Serial.println("leftPulses: "+String(leftPulses));
      Serial.println("rpmRL: "+String(rpmRL));
      Serial.println("leftSpeed: "+String(leftSpeed));
      leftPulses = 0;
    }
 // calcSpeedRight();
    RR = digitalRead(rightEncoderPin);  //FR = bitRead(PINE, 6);    
    Serial.println("RR: "+String(RR));
    if(RR == 0 && RRLast == 1) {
      leftPulses++;
      RRLast = RR;
    }else {
      RRLast = RR;  
    }  
    rightCurrentTime = millis() - rightLastTime;
    if(rightCurrentTime >= Interval){
      double Revelution = (double)rightPulses / Slots;
      rightSpeed = Revelution*(2*PI*Radiuse) / (rightCurrentTime/1000);
      rpmRR = Revelution*60/(rightCurrentTime/1000);
      rightLastTime = millis();
      Serial.println("rightPulses: "+String(rightPulses));
      Serial.println("rpmRR: "+String(rpmRL));
      Serial.println("rightSpeed: "+String(rightSpeed));
      rightPulses = 0;
    }
  sdTime = millis() - sdLastTime;
  if(sdTime > sdMarginTime){
    sdLastTime = millis();
    MyFile = SD.open("Data.txt", FILE_WRITE);  
    MyFile.println(String(millis())+","+String(FRSpeed)+","+String(FLSpeed)+","+String(rightSpeed)+","+String(leftSpeed)+","+String(accelerationX)+","+String(accelerationY)+","+String(accelerationZ)+","+String(gyroX - gyroBiasX)+","+String(gyroY - gyroBiasY)+","+String(gyroZ - gyroBiasZ)+","+String(magX - magBiasX)+","+String(magY - magBiasY)+","+String(magZ - magBiasZ)+","+String(mpu.getYaw()));
    MyFile.close();
    
    // the Direction of the wheel
    Direction = digitalRead(directionPin);
    if(digitalRead(Direction)){
      t0.setText("F");
    }
    else{
      t0.setText("R");
    }
    kmh.setValue((FLSpeed+FRSpeed)/2.0);
    rbm.setValue((rpmFL+rpmFR)/2.0);
    
    FRX.setValue(FRSpeed);
    FLX.setValue(FLSpeed);
    RRX.setValue(rightSpeed);
    RLX.setValue(leftSpeed);
    Left = digitalRead(leftSignal);
    if(!Left){
      lastLeft ^= 1;
      left.setValue(lastLeft);
    }
    else{
      left.setValue(0);  
    }
    Right =digitalRead(rightSignal);
    if(!Right){
      lastRight ^= 1; 
      right.setValue(lastRight);
    }
    else{
      right.setValue(0);
    }
    timex();
  }

  FR = bitRead(PINE, 6);    
  Serial.println("FR: "+String(FR));
  if(FR == 0 && FRLast == 1) {
    FRPulses++;
    FRLast = FR;
  }else {
    FRLast = FR;  
  }
  FRCurrentTime = millis() - FRLastTime;
  if(FRCurrentTime > FInterval){
    FRLastTime = millis();  
    double Revelution;
    Revelution = (double)FRPulses / (double)Slots;
    //FRSpeed = Revelution*(2*PI*Radiuse)*(18.0/5.0) / (FRCurrentTime/1000);
    rpmFR = Revelution*60/(FRCurrentTime/1000);
    //Serial.println("Pulses: "+String(FRPulses));
    //Serial.println("Speed: "+String(FRSpeed)+"km/h");
    //Serial.println("Speed: "+String(rpmFR)+"rpm");
    FRPulses = 0;
  }

  Serial.println("FL: "+String(FL));
  FL = bitRead(PINE, 7);
  if(FL == 0 && FLLast == 1) {
    FLPulses++;
    FLLast = FL;
  }
  else {
    FLLast = FL;  
  }
  FLCurrentTime = millis() - FLLastTime;
  if(FLCurrentTime > FInterval){
    FLLastTime = millis();
    double Revelution;
    Revelution = (double)FLPulses / Slots;
    FLSpeed = Revelution*(2*PI*Radiuse) / (FLCurrentTime/1000);
    rpmFL = Revelution*60/(FLCurrentTime/1000);
    Serial.println("Pulses: "+String(FLPulses));
    Serial.println("Speed: "+String(FLSpeed)+"km/h");
    Serial.println("Speed: "+String(rpmFL)+"rpm");
    FLPulses = 0; 
  }
  /*mpuCurrentTime = millis() - mpuLastTime;
  if(mpuCurrentTime > mpuInterval){
      mpuLastTime = millis();
      mpuCurrentTime = 0;
      Serial.println("accelerationX: " + String(accelerationX));
      Serial.println("accelerationY: " + String(accelerationY));
      Serial.println("accelerationZ: " + String(accelerationZ));
      
      Serial.println("accelerationBiasX: " + String(accelerationBiasX));
      Serial.println("accelerationBiasY: " + String(accelerationBiasY));
      Serial.println("accelerationBiasZ: " + String(accelerationBiasZ));
      
      Serial.println("Total gyroX: " + String(gyroX - gyroBiasX));
      Serial.println("Total gyroY: " + String(gyroY - gyroBiasY));
      Serial.println("Total gyroZ: " + String(gyroZ - gyroBiasZ));
      
      Serial.println("gyroX: " + String(gyroX));
      Serial.println("gyroY: " + String(gyroY));
      Serial.println("gyroZ: " + String(gyroZ));
      
      Serial.println("gyroBiasX: " + String(gyroBiasX));
      Serial.println("gyroBiasX: " + String(gyroBiasY));
      Serial.println("gyroBiasZ: " + String(gyroBiasZ));
      
      Serial.println("Total magX: "+ String(magX - magBiasX));
      Serial.println("Total magY: "+ String(magY - magBiasY));
      Serial.println("Total magZ: "+ String(magZ - magBiasZ));
      
      Serial.println("magX: "+ String(magX));
      Serial.println("magY: "+ String(magY));
      Serial.println("magZ: "+ String(magZ));
      
      Serial.println("magBiasX: "+ String(magBiasX));
      Serial.println("magBiasY: "+ String(magBiasY));
      Serial.println("magBiasZ: "+ String(magBiasZ));
      
      Serial.println("Yaw: " + String(mpu.getYaw()));
    }*/
}
