#ifndef ROVPID_H
#define ROVPID_H

#include "Energia.h"

extern double depthSetpoint, depthOutput;

void rovPIDinit();

boolean rovPIDrun();

void rovPIDsetMode(int mode);
  
#endif
