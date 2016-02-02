#include "rovPID.h"
#include "PID_v1.h"
#include "MS5837.h"

//Define variables we'll be connecting PID to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters for PID
double Kp=2, Ki=5, Kd=1;
PID depthPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MS5837 depthSensor;

/*
 * Initializes PID controllers and associated sensors
 */
void rovPIDinit(){
   //turn the PID on
  depthPID.SetMode(AUTOMATIC);

  depthSensor.init(0); //gets passed i2c module number
  depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

/*
 * Call this function cyclically to update PID control loops
 */
void rovPIDrun(){
  //get new depth data
  depthSensor.readFastPoll(); //takes about 5ms to get a new reading, non-blocking
  
  //update sensor data
  
  //run PID 
  depthPID.Compute();
  
  //update outputs
}
