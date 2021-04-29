#include <Arduino.h>

  bool Direction = 0, Reset = 0;
  int Counter, Slots, Pulses = 0 , Resistance , Radiuse;
  double Voltage, Current, Speed = 0;
  unsigned long time = 0;
  char directionPin = 13 , counterPin = 12, decoderPin = 8, resetPin = 7, voltagePin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(directionPin ,INPUT);
  pinMode(counterPin   ,INPUT);
  pinMode(decoderPin   ,INPUT);
  pinMode(resetPin     ,INPUT);
  pinMode(voltagePin   ,INPUT);
   
}

double calcSpeed(){
    if(Pulses == Slots){
      time = millis() - time;
      Pulses %= Slots;
      Speed = (2*PI*Radiuse)/time;
    }
    return Speed;
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
  Pulses   += digitalRead(decoderPin);

  // the voltage & current come from the battery
  Voltage   = analogRead(voltagePin);
  Current   = Voltage / Resistance;

  calcSpeed();



  

}
