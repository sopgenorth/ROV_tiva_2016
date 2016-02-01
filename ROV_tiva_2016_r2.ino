#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include "rovCOM.h"
#include "rovMotorDriver.h"
#include "PID_v1.h"
#include "MS5837.h"

//Define pariables we'll be connecting PID to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters for PID
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MS5837 depthSensor;

/*
 * ROV intialization. 
 */
void setup() {
  byte mac[] = {0x00, 0x1A, 0xB6, 0x02, 0xF4, 0xCC};
  setupSync(mac, IPAddress(192, 168, 1, 217), 20, 4545);

  //initialize motor driver lib and set UART rate
  setupMotorDriver(500000);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  depthSensor.init(0); //gets passed i2c module number
  depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

/*
 * Main loop that is called continuously. 
 */
void loop(){
  //get new depth data
  depthSensor.readFast();
  
  //reads a packet if available. 
  boolean newSyncData = updateSync();
  
  //updates motor values if new sync data is available or if more than 50ms have passed
  updateAllMotors(newSyncData);
}



