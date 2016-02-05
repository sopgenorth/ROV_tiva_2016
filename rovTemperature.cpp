#include "rovTemperature.h"
#include "rovCOM.h"
#include "OneWire.h"

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

enum tempStates_t{
  startConversion,
  need250msDelay,
  needLongDelay,
  tempRead
};

tempStates_t tempState = startConversion;
byte i;
byte present = 0;
byte type_s;
byte cfg;
int16_t raw;
byte data[12];
byte addr[8];
float rovTempCelsius;
unsigned long startMillis = 0;

boolean rovTemperatureRun(){
  switch(tempState){
  case startConversion:
    present = 0;
    if ( !ds.search(addr)) {
      ds.reset_search();
      tempState = need250msDelay;
      startMillis = millis();
      return false;
    }

    // the first ROM byte indicates which chip
    switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      tempState = startConversion;
      //Serial.println("Device is not a DS18x20 family device.");
      return false;
      break;
    } 
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    tempState = needLongDelay;
    startMillis = millis();
    break;

  case need250msDelay:
    if((millis() - startMillis) > 250) {
      tempState = startConversion;
    }
    break;

  case needLongDelay:
    if((millis() - startMillis) > 800) {
      tempState = tempRead;
    }
    break;

  case tempRead:
    present = ds.reset();
    ds.select(addr); 
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } 
    else {
      cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    rovTempCelsius = (float)raw / 16.0;
    outGroup.rovTempCelsius = rovTempCelsius;
    tempState = startConversion;
    return true;
    break;

  default:
    return false;
    break;  
  }
  return false; 
}

