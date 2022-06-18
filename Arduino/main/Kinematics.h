#pragma once
#include "Config.h"
#include "Motor.h"

class Kinematics{
  public:
    Kinematics(Motor, Motor); //2WD
    // Kinematics(Motor, Motor, LED, LED); //2WD with 2 LEDs
    void Move(int lpwm, int rpwm); //actuate both motors according to pwm/direction
    int isRosConnected;
  private:
    Motor LH_motor;
    Motor RH_motor;
};

Kinematics::Kinematics(Motor LH_motor, Motor RH_motor)
: LH_motor(LH_motor), RH_motor(RH_motor)
{

}

void Kinematics::Move(int lpwm, int rpwm){
  //add "-" sign to invert direction if needed
  this->LH_motor.Rotate(lpwm);
  this->RH_motor.Rotate(rpwm);
}
