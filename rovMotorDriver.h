#ifndef ROV_MOTOR_DRIVER_H
#define ROV_MOTOR_DRIVER_H
#include "Energia.h"

#define TOTAL_THRUSTERS 8

void setupMotorDriver(int uartRate);
void stopAllMotors();
void setMotorSpeed(int serialPort, int16_t motorvalue);
void updateAllMotors(boolean overRideTimer);


#endif
