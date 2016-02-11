#include "Analog.h"
#include "rovCOM.h"

#define ANALOG_SAMPLE_TIME_MS 50

#define rov48V_pin PE_0
#define rov48I_pin PD_7
#define rov12V_pin PD_6 

void rovAnalogInit(){
  rovAnalogSample();
}

void rovAnalogSample(){
  unsigned long currentMillis = millis();
  static unsigned long previousMillis;
  
 // if(currentMillis - previousMillis > ANALOG_SAMPLE_TIME_MS) {
    previousMillis = currentMillis;
    outGroup.rov48V = analogRead(rov48V_pin);
    outGroup.rov48I = analogRead(rov48I_pin);
    outGroup.rov12V = analogRead(rov12V_pin);
  //}
}
