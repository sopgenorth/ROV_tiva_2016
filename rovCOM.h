#ifndef rovCOM_H
#define rovCOM_H

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define IN_NUM 8
//data received from PC
struct inNames_t{
  int32_t m0,m1,m2,m3,m4,m5,m6,m7; //8 motors/thrusters, speed range: -3200 to 3200
};

#define OUT_NUM 3
//data to send to PC
struct outNames_t{
  int32_t depth_microBar, rovTempCelsius, o3;
};

extern inNames_t inGroup;
extern outNames_t outGroup;


#define DEBUG false

void setupSync(byte mac[], IPAddress ip, int updateRate, int localPort);
int updateSync();

#endif
