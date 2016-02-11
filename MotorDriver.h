#ifndef ROV_MOTOR_DRIVER_H
#define ROV_MOTOR_DRIVER_H
#include "Energia.h"

#define TOTAL_THRUSTERS 8
#define br_xy 0
#define bl_xy 5
#define fl_xy 1
#define fr_xy 2

#define fr_z 3
#define fl_z 7
#define br_z 4
#define bl_z 6

extern const int thrusterNumbers[8];

void setupMotorDriver(int uartRate);
void stopAllMotors();
void setMotorSpeed(int serialPort, int16_t motorvalue);
void updateAllMotors(boolean overRideTimer);


#endif
