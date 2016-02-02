#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include "rovCOM.h"
#include "rovMotorDriver.h"
#include "rovPID.h"

/*
 * ROV intialization. 
 */
void setup() {
  byte mac[] = {
    0x00, 0x1A, 0xB6, 0x02, 0xF4, 0xCC  };
  setupSync(mac, IPAddress(192, 168, 1, 217), 20, 4545);

  //initialize motor driver lib and set UART rate
  setupMotorDriver(500000);
  
  //start PID control
  rovPIDinit();
}

/*
 * Main loop that is called continuously. 
 */
void loop(){
  //reads a packet if available. 
  boolean newSyncData = updateSync();

  //updates motor values if new sync data is available or if more than 50ms have passed
  updateAllMotors(newSyncData);
  
  //update PID loops
  rovPIDrun();
}




