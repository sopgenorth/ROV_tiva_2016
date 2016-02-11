#include "PID.h"
#include "PID_v1.h"
#include "MS5837.h"
#include "rovCOM.h"

#define PID_LOOP_RATE_MS 7
#define PID_GAIN_SWITCH_POINT (1500)
int32_t PIDgainSwitchPoint_uBar = PID_GAIN_SWITCH_POINT;

//Define variables we'll be connecting PID to
double depthSetpoint, depthInput, depthOutput;
int32_t zThrusters;

//Specify the links and initial tuning parameters for PID
int32_t Kp=215000, Ki=68500, Kd=88000;
int32_t Kp_low=270, Ki_low=350, Kd_low=150;
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
boolean highGains = true;
boolean rovPIDrun(){
  //get new depth data
  depthSensor.read(); //takes about 4.7ms to get a new reading, non-blocking
  
  //update sensor data
  depthInput = depthSensor.pressure();
  
  //don't compute PID unless told to
  if(inGroup.PIDenable == 0){
    return false; 
  }
  
  if(inGroup.PID_setPoint != 0 && inGroup.PID_setPoint != prevPID_setPoint){
    prevPID_setPoint = inGroup.PID_setPoint;
    depthSetpoint = (double) inGroup.PID_setPoint;
  }
    
  if((inGroup.PIDgainSwitchPoint_uBar != 0 && inGroup.PIDgainSwitchPoint_uBar != PIDgainSwitchPoint_uBar)){
    PIDgainSwitchPoint_uBar = inGroup.PIDgainSwitchPoint_uBar;
  }
  
  if(abs(depthInput - depthSetpoint) > (PIDgainSwitchPoint_uBar/1000.0f)){ //far away from setpoint, use the high gains
    if(!highGains || (inGroup.PID_Kp != 0 && inGroup.PID_Kp != Kp)){
      Kp = inGroup.PID_Kp;
      depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
    }
    if(!highGains || (inGroup.PID_Ki != 0 && inGroup.PID_Ki != Ki)){
      Ki = inGroup.PID_Ki;
      depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
    }
    if(!highGains || (inGroup.PID_Kd != 0 && inGroup.PID_Kd != Kd)){
      Kd = inGroup.PID_Kd;
      depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
    }
    highGains = true;
  }
  else //we're pretty close to the setpoint, use lower gains
  {
    if(highGains || (inGroup.PID_Kp_low != 0 && inGroup.PID_Kp_low != Kp_low)){
      Kp_low = inGroup.PID_Kp_low;
      depthPID.SetTunings(Kp_low/1000.0f, Ki_low/1000.0f, Kd_low/1000.0f);
    }
    if(highGains || (inGroup.PID_Ki_low != 0 && inGroup.PID_Ki_low != Ki_low)){
      Ki_low = inGroup.PID_Ki_low;
      depthPID.SetTunings(Kp_low/1000.0f, Ki_low/1000.0f, Kd_low/1000.0f);
    }
    if(highGains || (inGroup.PID_Kd_low != 0 && inGroup.PID_Kd_low != Kd_low)){
      Kd_low = inGroup.PID_Kd_low;
      depthPID.SetTunings(Kp/1000.0f, Ki/1000.0f, Kd/1000.0f);
    }
    highGains = false;
  }
  
  boolean updatedValues = false;
  
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
