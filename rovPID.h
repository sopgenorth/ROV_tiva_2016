#ifndef ROVPID_H
#define ROVPID_H

#include "Energia.h"

extern double depthSetpoint, depthOutput;
extern int32_t zThrusters;

void rovPIDinit();

boolean rovPIDrun();

void rovPIDsetMode(int mode);
  
#endif
