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
    // if(!robotState->isEnabled()){
    //     armServo->write(armSetAngle);
    // }
    // update from action controller
    switch (robotState->getNextAction()){
        case STOP:
            home();
            break;
        case INTAKE:
            home(); // we want mechanisms to automatically go to the position to intake when the intake button is pressed, not when execute is pressed
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
        case SOURCE:
            armMode = 7;
            armSetAngle = arm_SOURCE_angle;
            // execute(); // we want mechanisms to automatically go to the position to intake when the intake button is pressed, not when execute is pressed
            break;
        case CLIMBERS_UP:
            armMode = 8;
            armSetAngle = arm_CLIMB_DEPLOY_angle;
            execute(); // we want the arm to automatically go to the position to climb when the climb button is pressed, not when execute is pressed
            break;
        case CLIMBERS_DOWN:
            armMode = 9;
            armSetAngle = arm_CLIMB_RETRACT_angle;
            execute(); // we want the arm to automatically go to the position to climb when the climb button is pressed, not when execute is pressed
            break;
        default:
            break;
    }
    return armMode;
}

void Arm::execute(){
    if(robotState->isEnabled()){
        armServo->write(armSetAngle);
    }

}

void Arm::home(){
    armMode = 0;
    armSetAngle = arm_STOW_angle;
    execute();
}

void Arm::setDynamic(float angle){
    armMode = 6; // dynamic mode
    armSetAngle = angle;
    execute();
}