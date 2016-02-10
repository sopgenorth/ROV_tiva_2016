#include "MS5837.h"
//#include <Wire.h>
#include "I2CMaster.h"
#include "rovCOM.h"

#define MS5837_ADDR               0xEC  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1         0x46
#define MS5837_CONVERT_D2         0x56

#if (MS5837_CONVERT_D1 == 0x40)
# define MS5837_CONVERT_DELAY_D1 600
#elif (MS5837_CONVERT_D1 == 0x42)
# define MS5837_CONVERT_DELAY_D1 1170
#elif (MS5837_CONVERT_D1 == 0x44)
# define MS5837_CONVERT_DELAY_D1 2280
#elif (MS5837_CONVERT_D1 == 0x46)
# define MS5837_CONVERT_DELAY_D1 4540
#elif (MS5837_CONVERT_D1 == 0x48)
# define MS5837_CONVERT_DELAY_D1 9040
#else
# error "Invalid MS5837_CONVERT_D1 value"
#endif

#if (MS5837_CONVERT_D2 == 0x50)
# define MS5837_CONVERT_DELAY_D2 600
#elif (MS5837_CONVERT_D2 == 0x52)
# define MS5837_CONVERT_DELAY_D2 1170
#elif (MS5837_CONVERT_D2 == 0x54)
# define MS5837_CONVERT_DELAY_D2 2280
#elif (MS5837_CONVERT_D2 == 0x56)
# define MS5837_CONVERT_DELAY_D2 4540
#elif (MS5837_CONVERT_D2 == 0x58)
# define MS5837_CONVERT_DELAY_D2 9040
#else
# error "Invalid MS5837_CONVERT_2 value"
#endif


/*
 * It probably isn't important to read the temperature
 * from the depth sensor every time since temperature should 
 * change relatively slowly. This sets how many pressure reads
 * are between temperature readings. 
 */
#define TEMP_READ_RATE 50  

#define SCL_PIN PK_3
#define SDA_PIN PK_2

SoftI2cMaster rtc(SDA_PIN, SCL_PIN);



MS5837::MS5837() {
  fluidDensity = 1029;
}

void MS5837::init() {
  // Reset the MS5837, per datasheet
  delayMicroseconds(5);
  rtc.start(MS5837_ADDR | I2C_WRITE);

  delayMicroseconds(5);
  rtc.write(MS5837_RESET);
  delayMicroseconds(5);
  //Serial.println(Wire.endTransmission());
  rtc.stop();
  
  Serial.println("reset done");

  // Wait for reset to complete
  delay(10);

  // Read calibration values and CRC
  for ( uint8_t i = 0 ; i < 8 ; i++ ) {
    rtc.start(MS5837_ADDR | I2C_WRITE);
    rtc.write(MS5837_PROM_READ+i*2);
    rtc.stop();

    rtc.start(MS5837_ADDR | I2C_READ);
    byte b1 = rtc.read(false);
    byte b2 = rtc.read(true);
    rtc.stop();
    C[i] = (b1 << 8) | b2;
  }

  // Verify that data is correct with CRC
  uint8_t crcRead = C[0] >> 12;
  uint8_t crcCalculated = crc4(C);

  if ( crcCalculated == crcRead ) {
    // Success
  } 
  else {
    // Failure - try again?
  }
  
  //attempt a first read to get a pressure basis
  depthSampleCounter = TEMP_READ_RATE; //forces first read to read the temperature
  read(); //this will read the current temperature
  read(); //this will read the current pressure
  prevPressure = P;
  
  
}

void MS5837::setFluidDensity(float density) {
  fluidDensity = density;
}

void MS5837::read() {    
  if(depthSampleCounter < TEMP_READ_RATE){ //read the pressure
    depthSampleCounter++;
  
    // Request D1 conversion
    rtc.start(MS5837_ADDR | I2C_WRITE);
    rtc.write(MS5837_CONVERT_D1);
    rtc.stop();
  
    delayMicroseconds(MS5837_CONVERT_DELAY_D1); // Max conversion time per datasheet
  
    rtc.start(MS5837_ADDR | I2C_WRITE);
    rtc.write(MS5837_ADC_READ);
    rtc.stop();
    
    rtc.start(MS5837_ADDR | I2C_READ);
    D1 = 0;
    D1 = rtc.read(false);
    D1 = (D1 << 8) | rtc.read(false);
    D1 = (D1 << 8) | rtc.read(true);
    rtc.stop();
  }
  else { //read the temperature every TEMP_READ_RATE depth readings
    depthSampleCounter = 0;
  
    // Request D2 conversion
    rtc.start(MS5837_ADDR | I2C_WRITE);
    rtc.write(MS5837_CONVERT_D2);
    rtc.stop();
    delayMicroseconds(MS5837_CONVERT_DELAY_D2);
  
    rtc.start(MS5837_ADDR | I2C_WRITE);
    rtc.write(MS5837_ADC_READ);
    rtc.stop();
  
    rtc.start(MS5837_ADDR | I2C_READ);
    D2 = 0;
    D2 = rtc.read(false);
    D2 = (D2 << 8) | rtc.read(false);
    D2 = (D2 << 8) | rtc.read(true);
    rtc.stop();
    
  }
  calculate();
}

void MS5837::readTestCase() {
  C[0] = 0;
  C[1] = 34982;
  C[2] = 36352;
  C[3] = 20328;
  C[4] = 22354;
  C[5] = 26646;
  C[6] = 26146;
  C[7] = 0;

  D1 = 4958179;
  D2 = 6815414;

  calculate();
}

void MS5837::calculate() {
  // Given C1-C6 and D1, D2, calculated TEMP and P
  // Do conversion first and then second order temp compensation

  int32_t dT;
  int64_t SENS;
  int64_t OFF;
  int32_t SENSi; 
  int32_t OFFi;  
  int32_t Ti;    
  int64_t OFF2;
  int64_t SENS2;

  // Terms called
  dT = D2-uint32_t(C[5])*256l;
  SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
  OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;


  //Temp and P conversion
  TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
  P = (D1*SENS/(2097152l)-OFF)/(32768l);

  //Second order compensation
  if((TEMP/100)<20){         //Low temp
    Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
    OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
    SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
    if((TEMP/100)<-15){    //Very low temp
      OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
      SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
    }
  }
  else if((TEMP/100)>=20){    //High temp
    Ti = 2*(dT*dT)/(137438953472LL);
    OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
    SENSi = 0;
  }

  OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
  SENS2 = SENS-SENSi;

  TEMP = (TEMP-Ti);
  P = (((D1*SENS2)/2097152l-OFF2)/32768l);
}

float MS5837::pressure(float conversion) {
  depthFilter = inGroup.depthFilter;
  
  float filterN = depthFilter/1000.0f;
  
  //apply an exponential weighted moving average filter to remove sensor jitter
  prevPressure = (prevPressure*filterN) + ((P/10.0f)*(1.0f-filterN)); 
  //update data to send over ethernet
  outGroup.depth_microBar = (int32_t) (prevPressure * 1000.0f);
  outGroup.depth_microBarRaw = (int32_t) (P*100.0f);
  return prevPressure*conversion;
}

float MS5837::temperature() {
  return TEMP/100.0f;
}

float MS5837::depth() {
  return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837::altitude() {
  return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for ( uint8_t i = 0 ; i < 16; i++ ) {
    if ( i%2 == 1 ) {
      n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF0);
    } 
    else {
      n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
    }
    for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
      if ( n_rem & 0x8000 ) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } 
      else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = ((n_rem >> 12) & 0x000F);

  return n_rem ^ 0x00;
}

