#include "rovMotorDriver.h"
#include "rovCOM.h"
#include "rovPID.h"

#define m_DIR PK_5
#define m_PWM PF_0

const int thrusterNumbers[8] = {fl_xy, fl_z, fr_xy, fr_z, br_xy, br_z, bl_xy, bl_z}; 

const unsigned char CRC7_POLY = 0x91;
unsigned char CRCTable[256];

//internal function prototypes
unsigned char getCRC(unsigned char message[], unsigned char length);
unsigned char getCRCForByte(unsigned char val);
void buildCRCTable();

/*
 * Begin serial ports, built CRC table and send safe start command 
 * to the motor controllers. 
 */
void setupMotorDriver(int uartRate)
{
  //ROV uses all 8 hardware UARTs for motor control
  Serial.begin(uartRate);
  Serial1.begin(uartRate);
  Serial2.begin(uartRate);
  Serial3.begin(uartRate);
  Serial4.begin(uartRate);
  Serial5.begin(uartRate);
  Serial6.begin(uartRate);
  Serial7.begin(uartRate);

  delay(5); //wait for motor drivers to initialize

  //Build CRC table used for data integretity verification
  buildCRCTable();

  //Motor controller signal for auto baud identification
  unsigned char message[2] = {
    0xAA, 0x00      };
  message[1] = getCRC(message, 1);
  Serial.write(message, sizeof(message));
  Serial1.write(message, sizeof(message));
  Serial2.write(message, sizeof(message));
  Serial3.write(message, sizeof(message));
  Serial4.write(message, sizeof(message));
 // Serial5.write(message, sizeof(message));
  Serial6.write(message, sizeof(message));
  Serial7.write(message, sizeof(message));

  //command for exiting motor controller safe start
  message[0] = 0x83;
  message[1] = getCRC(message, 1);
  Serial.write(message, sizeof(message));
  Serial1.write(message, sizeof(message));
  Serial2.write(message, sizeof(message));
  Serial3.write(message, sizeof(message));
  Serial4.write(message, sizeof(message));
  //Serial5.write(message, sizeof(message));
  Serial6.write(message, sizeof(message));
  Serial7.write(message, sizeof(message));

  //for added motor driver
  pinMode(m_DIR, OUTPUT);
  pinMode(m_PWM, OUTPUT);

  //bring all motors to a stop
  stopAllMotors(); 
  rovPIDinit();
}

/*
 * Updates the speed of a chosen motor. Value should be in the 
 * range -3200 to 3200, and will be clipped if it exceeds. 
 */
void setMotorSpeed(int serialPort, int16_t motorvalue)
{
  unsigned char buf[4];
  if (motorvalue < 0){
    buf[0] = 0x86;  // motor reverse command
    motorvalue = -motorvalue;  // make speed positive
  }
  else {
    buf[0] = 0x85;  // motor forward command
  }

  //limit motor max value to 3200
  if (motorvalue > 3200) {
    motorvalue = 3200;    
  }

  //bit work for preparing the value to be sent
  buf[1] = motorvalue & 0x1F;
  buf[2] = (motorvalue >> 5);

  //add the CRC error checking to the end of the message
  buf[3] = getCRC(buf, 3);

  //send the updated motor value to the proper motor
  switch(serialPort){
  case 0:
    Serial.write(buf, sizeof(buf));
    break;
  case 1:
    Serial1.write(buf, sizeof(buf));
    break;
  case 2:
    Serial2.write(buf, sizeof(buf));
    break;
  case 3:
    Serial3.write(buf, sizeof(buf));
    break;
  case 4:
    Serial4.write(buf, sizeof(buf));
    break;
  case 5:
    //Serial5.write(buf, sizeof(buf));
    //needed to swap out motor driver...
    //motorvalue has already been stripped of its sign
    //this is the only way to know dir here
    if (buf[0] == 0x86){
      digitalWrite(m_DIR, HIGH);
      analogWrite(m_PWM, map(motorvalue, 0, 3200, 0, 255));
    }
    else {
      digitalWrite(m_DIR, LOW);
      analogWrite(m_PWM, map(motorvalue, 0, 3200, 0, 255));
    }
    break;
  case 6:
    Serial6.write(buf, sizeof(buf));
    break;
  case 7: 
    Serial7.write(buf, sizeof(buf));
    break;
  }
}

/*
 * Brings all motors to an immediate stop.
 */
void stopAllMotors()
{
  for(int i = 0; i <  TOTAL_THRUSTERS; i++){
    setMotorSpeed(i, 0); 
  }
}

/*
 * Simple helper function to update all motor values.
 * Updates motor values with data from rovCOM handler
 * Only sends serial data if 50ms has passed or if passed 'true'
 */
void updateAllMotors(boolean overRideTimer){
  unsigned long currentMillis = millis();
  static unsigned long previousMillis;
  
  if(overRideTimer || (currentMillis - previousMillis > 50)) {
    previousMillis = currentMillis;
    
    //update motor values using received data
    int * motorValues = (int*)&inGroup;
    for(int i = 0; i < TOTAL_THRUSTERS; i+=2){
      setMotorSpeed(thrusterNumbers[i], *motorValues);
      motorValues++; 
    }
  }
  
  //integrate z thrust data and update depthSetpoint
  
  //call the PID run function and update z thrustrs if necessary
  boolean updatedPID = rovPIDrun();
  if(updatedPID){
     //update 4 Z thrusters
     for(int i = 1; i < TOTAL_THRUSTERS; i+=2){
      setMotorSpeed(thrusterNumbers[i], zThrusters);
    }
    outGroup.PID_output = zThrusters;
  }
  
}

unsigned char getCRCForByte(unsigned char val)
{
  unsigned char j;

  for (j = 0; j < 8; j++)
  {
    if (val & 1)
      val ^= CRC7_POLY;
    val >>= 1;
  }

  return val;
}

void buildCRCTable()
{
  int i;

  // fill an array with CRC values of all 256 possible bytes
  for (i = 0; i < 256; i++)
  {
    CRCTable[i] = getCRCForByte(i);
  }
}

unsigned char getCRC(unsigned char message[], unsigned char length)
{
  unsigned char i, crc = 0;

  for (i = 0; i < length; i++)
    crc = CRCTable[crc ^ message[i]];
  return crc;
}

