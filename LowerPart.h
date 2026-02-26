#ifndef LOWERPART_H
#define LOWERPART_H

#include "Motor.h"

class LowerPart {
  private:
    Motor roda_1;
    Motor roda_2;
    Motor roda_3;
    Motor roda_4;
    // Motor roda_5; // slider motor

  public:
    LowerPart();   // dummy (pin 0,0)
    LowerPart(byte sel_1, byte pwm_1,
              byte sel_2, byte pwm_2,
              byte sel_3, byte pwm_3,
              byte sel_4, byte pwm_4); // lengkap

    void Movement(float V1, float V2, float V3, float V4);  // nilai −4095…+4095
    // void RollerMovement(float V5); // nilai −4095…+4095
};


class LowerPartMid {
  private:
    MotorMid roda_5;

  public:
    LowerPartMid();   // dummy (pin 0,0)
    LowerPartMid(byte rpwm_mid, byte lpwm_mid); // lengkap

    void RollerMovement(float V5);  // nilai −4095…+4095
};

#endif