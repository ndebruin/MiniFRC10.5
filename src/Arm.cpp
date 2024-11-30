#include <Arduino.h>
#include "Arm.h"

#include "Constants.h"

Arm::Arm(NoU_Servo* ArmServo, State* state) : armServo(ArmServo), robotState(state)
{ }

uint8_t Arm::begin(){
    armServo->write(arm_STOW_angle);

    return 0;
}

uint8_t Arm::update(){
    if(armServo->getDegrees() != armSetAngle){
        armServo->write(armSetAngle);
    }

    return 0;
}

uint8_t Arm::getMode(){
    return armMode;
}

void Arm::set(double ArmAngle){
    armMode = 1;
    armServo->write(ArmAngle);
}

void Arm::setMode(uint8_t position)
{
    
}

