#include "rovPID.h"
#include "PID_v1.h"
#include "MS5837.h"

//Define variables we'll be connecting PID to
double depthSetpoint, depthInput, depthOutput;

//Specify the links and initial tuning parameters for PID
double Kp=2, Ki=5, Kd=1;
PID depthPID(&depthInput, &depthOutput, &depthSetpoint, Kp, Ki, Kd, DIRECT);

MS5837 depthSensor;

/*
 * Initializes PID controllers and associated sensors
 */
void rovPIDinit(){
  //configure depth PID
  depthPID.SetOutputLimits(-3200,3200);
  depthPID.SetSampleTime(5); //sample at 5ms intervals
  
   //turn the PID on
  depthPID.SetMode(AUTOMATIC);

  //depthSensor.init(5); //gets passed i2c module number
 // depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

/*
 * Call this function cyclically to update PID control loops
 * Returns true if values were updated
 */
unsigned long previousTime = 0;
boolean rovPIDrun(){
  boolean updatedValues = false;
  
  //get new depth data
  depthSensor.readSlow(); //takes about 4.7ms to get a new reading, non-blocking
  
  //update sensor data
    depthInput = depthSensor.pressure();
    Serial.print("Depth is: ");
    Serial.println(depthInput);
    return true;
  
  //update depthPID every 5ms
  unsigned long currentTime = millis();
  if(currentTime - previousTime > 5) {
    previousTime = currentTime;
        
    //update sensor data
    depthInput = depthSensor.pressure();
    Serial.println(depthInput);
    
    //run PID 
    updatedValues = depthPID.Compute(true);
  }
  
  return updatedValues;
}

/*
 * Set automatic (non-zero) or manual (0) mode for the PID loops
 */
void rovPIDsetMode(int mode){
  depthPID.SetMode(mode);
}
