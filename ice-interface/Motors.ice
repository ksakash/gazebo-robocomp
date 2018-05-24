#ifndef ROBOCOMPMOTORS_ICE
#define ROBOCOMPMOTORS_ICE

// #include "common.ice"

module RoboCompMotors{

  class Motors
  {
    float getMotorSpeed();
    void setMotorSpeed(float w);
    float motorSpeed;
  };

}; 

#endif //MOTORS_ICE
