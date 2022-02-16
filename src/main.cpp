#include <Arduino.h>

  bool Direction = 0, Reset = 0;
  int Counter, Slots, leftPulses = 0 , rightPulses = 0 , Resistance , Radiuse , totalLaps;
  double Voltage, Current, leftSpeed = 0.0 , rightSpeed = 0.0,Time = 0.0,usedCurrent = 0.0, SOC = 0.0, TotalCharge = 0;
  unsigned long currentTime, lastTime , Interval = 0, socInterval = 1, socCurrentTime = 0, socLastTime = 0;
  char directionPin = 13 , counterPin = 12, leftEncoderPin = 2,rightEncoderPin = 3, resetPin = 7, voltagePin = A0;

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
