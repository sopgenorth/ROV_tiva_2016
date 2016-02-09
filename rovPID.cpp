#include "rovPID.h"
#include "PID_v1.h"
#include "MS5837.h"
#include "rovCOM.h"

#define PID_LOOP_RATE_MS 7

//Define variables we'll be connecting PID to
double depthSetpoint, depthInput, depthOutput;
int32_t zThrusters;

//Specify the links and initial tuning parameters for PID
int32_t Kp=85000, Ki=79000, Kd=65000;
PID depthPID(&depthInput, &depthOutput, &depthSetpoint, Kp/1000.0f, Ki/1000.0f, Kd/1000.0f, REVERSE);

MS5837 depthSensor;

/*
 * Initializes PID controllers and associated sensors
 */
void rovPIDinit(){
  //configure depth PID
  depthPID.SetOutputLimits(-3200,3200);
  depthPID.SetSampleTime(PID_LOOP_RATE_MS); //sample at 5ms intervals
  
   //turn the PID on
  depthPID.SetMode(AUTOMATIC);

  depthSensor.init(); 
  depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  depthSensor.read();
  depthSetpoint = depthSensor.pressure();
}

/*
 * Call this function cyclically to update PID control loops
 * Returns true if values were updated
 */
unsigned long previousTime = 0;
int32_t prevPID_setPoint = 0;
boolean rovPIDrun(){
  if(inGroup.PID_setPoint != 0 && inGroup.PID_setPoint != prevPID_setPoint){
    prevPID_setPoint = inGroup.PID_setPoint;
    depthSetpoint = (double) inGroup.PID_setPoint;
  }
  
  if(inGroup.PID_Kp != 0 && inGroup.PID_Kp != Kp){
    Kp = inGroup.PID_Kp;
    depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
  }
  if(inGroup.PID_Ki != 0 && inGroup.PID_Ki != Ki){
    Ki = inGroup.PID_Ki;
    depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
  }
  if(inGroup.PID_Kd != 0 && inGroup.PID_Kd != Kd){
    Kd = inGroup.PID_Kd;
    depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
  }
  
  boolean updatedValues = false;
  
  //get new depth data
  depthSensor.read(); //takes about 4.7ms to get a new reading, non-blocking
  
  //update sensor data
    depthInput = depthSensor.pressure();
  
  //update depthPID every PID_LOOP_RATE_MS
  unsigned long currentTime = millis();
  if(currentTime - previousTime > PID_LOOP_RATE_MS) {
    previousTime = currentTime;
    
    //run PID 
    updatedValues = depthPID.Compute(true);
    zThrusters = depthOutput;
    updatedValues = true;
  }
  
  return updatedValues;
}

/*
 * Set automatic (non-zero) or manual (0) mode for the PID loops
 */
void rovPIDsetMode(int mode){
  depthPID.SetMode(mode);
}
