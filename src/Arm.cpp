#include <Arduino.h>
#include "Arm.h"

#include "Constants.h"

Arm::Arm(NoU_Servo* ArmServo, State* state) : armServo(ArmServo), robotState(state)
{ }

uint8_t Arm::begin(){
    armServo->write(arm_STOW_angle);
    return 0;
}

int8_t Arm::update(){
    // update from action controller
    switch (robotState->getNextAction()){
        case STOP:
            armMode = 0;
            armSetAngle = arm_STOW_angle;
            break;
        case SUBWOOFER:
            armMode = 2;
            armSetAngle = arm_SUB_angle;
            break;
        case AMP_FORWARD:
            armMode = 3;
            armSetAngle = arm_AMP_FORWARD_angle;
            break;
        case AMP_BACKWARD:
            armMode = 4;
            armSetAngle = arm_AMP_BACKWARD_angle;
            break;
        case PASS:
            armMode = 5;
            armSetAngle = arm_PASS_angle;
            break;
        case DYNAMIC:
            armMode = 6;
            // this one is weird, we'll see
            break;
        case SOURCE:
            armMode = 7;
            armSetAngle = arm_SOURCE_angle;
            break;
        case CLIMBERS_UP:
            armMode = 8;
            armSetAngle = arm_CLIMB_DEPLOY_angle;
            break;
        case CLIMBERS_DOWN:
            armMode = 9;
            armSetAngle = arm_CLIMB_RETRACT_angle;
            break;
        case -1: // custom angle, do nothing
            break;
    }
    return armMode;
}

void Arm::execute(){
    if(robotState->isEnabled()){
        if(armServo->getDegrees() != armSetAngle){
            armServo->write(armSetAngle);
        }
    }
}