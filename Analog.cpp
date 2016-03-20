#include "Analog.h"
#include "rovCOM.h"

#define ANALOG_SAMPLES_PER_SCAN 5

#define rov48V_pin PE_0
#define rov48I_pin PD_7
#define rov12V_pin PD_6 

//simple moving average filter
#define FILTER_VALUE_COUNT 25
class  FilterMA
{
public:
  FilterMA()
  {
    for(int i=0; i < FILTER_VALUE_COUNT; i++)
      v[i]=0;
  }
private:
  int32_t v[FILTER_VALUE_COUNT];
public:
  int32_t step(int32_t x) //class II 
  {
    int32_t average = 0;
    for(int i=0; i< (FILTER_VALUE_COUNT-1); i++){
      v[i] = v[i+1]; //shift the values over
      average += v[i]; //sum the past
    }
    v[FILTER_VALUE_COUNT - 1] = x;
    average += x;
    return (average / FILTER_VALUE_COUNT);
  }
};

FilterMA rov48V_filter, rov48I_filter, rov12V_filter;

void rovAnalogInit(){
  rovAnalogSample();
}

void rovAnalogSample(){
  int32_t rov48V = 0, rov48I = 0, rov12V = 0;

  for(int i = 0; i < ANALOG_SAMPLES_PER_SCAN; i++){
      rov48V += analogRead(rov48V_pin);
      rov48I += analogRead(rov48I_pin);
      rov12V += analogRead(rov12V_pin);
  }
  
  //apply a moving average filter to the data
  outGroup.rov48V = rov48V_filter.step(rov48V / ANALOG_SAMPLES_PER_SCAN);
  outGroup.rov48I = rov48I_filter.step(rov48I / ANALOG_SAMPLES_PER_SCAN); 
  outGroup.rov12V = rov12V_filter.step(rov12V / ANALOG_SAMPLES_PER_SCAN);
}
