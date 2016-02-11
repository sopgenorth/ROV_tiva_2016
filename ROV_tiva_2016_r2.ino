
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>

#include "rovCOM.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Temp.h"
#include "Analog.h"


/*
 * ROV intialization. 
 */
void setup() {
  
//this needs to match the board's MAC for Ethernet firmware updates to work  
byte mac[] = {0x00, 0x1A, 0xB6, 0x02, 0xF4, 0xCC};
  setupSync(mac, IPAddress(192, 168, 2, 217), 20, 4545);
  
  //initialize motor driver lib and set UART rate
  setupMotorDriver(500000);
}

/*
 * Main loop that is called continuously. 
 */
void loop(){
  unsigned long startTime = micros();
  
  //reads a packet if available. 
  boolean newSyncData = updateSync();
  
  //updates motor values if new sync data is available or if more than 50ms have passed
  updateAllMotors(newSyncData);

  //run the temperature poll
  rovTemperatureRun();
  
  //sample ADC values
  rovAnalogSample();
  
  outGroup.rovCycleTime_us = (int32_t) (micros()-startTime);
}





