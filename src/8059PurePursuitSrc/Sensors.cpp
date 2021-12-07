#include "main.h"
/** global variables */
Node position;
double encdL = 0, encdR = 0, bearing = 0, angle = halfPI;
double measuredV = 0, measuredVL = 0, measuredVR = 0;
// angle = mathematical angle, taken from x-axis, counter clockwise as positive
void Sensors(void * ignore){
  // port data from all sensors
  Motor FL (FLPort);
  Motor BL (BLPort);
  Motor FR (FRPort);
  Motor BR (BRPort);
  Imu imu(imuPort);
  ADIEncoder encoderL(encdL_port, encdL_port+1, false);
  ADIEncoder encoderR(encdR_port, encdR_port+1, false);
  bool calibrated = false;
  int start = millis();
  while(true){
    encdL = encoderL.get_value();
    encdR = encoderR.get_value();
    bearing = imu.is_calibrating()? 0 : -imu.get_rotation();
    angle = boundRad(halfPI - bearing*toRad);
    measuredVL = (FL.get_actual_velocity() + BL.get_actual_velocity())/2 * RPMToInPerMs;
    measuredVR = (FR.get_actual_velocity() + BR.get_actual_velocity())/2 * RPMToInPerMs;;
    measuredV = (measuredVL + measuredVR)/2;
    delay(dT);
  }
}
