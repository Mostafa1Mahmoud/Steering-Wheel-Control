#include <Arduino.h>

  bool Direction = 0, Reset = 0;
  int Counter, Slots, Pulses = 0 , Resistance , Radiuse;
  double Voltage, Current, Speed = 0;
  unsigned long currentTime, lastTime , Interval = ;
  char directionPin = 13 , counterPin = 12, encoderPin = 8, resetPin = 7, voltagePin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(directionPin ,INPUT);
  pinMode(counterPin   ,INPUT);
  pinMode(encoderPin   ,INPUT);
  pinMode(resetPin     ,INPUT);
  pinMode(voltagePin   ,INPUT);
   
}

void calcSpeed(){
    double Revelution;
    currentTime = millis() - lastTime;
    if(currentTime >= Interval){
      Revelution = (double)Pulses / Slots;
      Slots = 0;
      Speed = Revelution*(2*PI*Radiuse) / currentTime;
      lastTime = currentTime;
    }
}


void loop() {
  // put your main code here, to run repeatedly:
  // the Direction of the wheel
  Direction = digitalRead(directionPin);
  
  // the reset button
  Reset     = digitalRead(resetPin);
  
  // the lab button
  Counter  += digitalRead(counterPin);
  
  // the number of bulses for calculating the speed
  Pulses   += digitalRead(encoderPin);

  // the voltage & current come from the battery
  Voltage   = analogRead(voltagePin);
  Current   = Voltage / Resistance;

  calcSpeed();



  

}
