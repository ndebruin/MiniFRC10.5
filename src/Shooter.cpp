#include <Arduino.h>
#include "Shooter.h"

Shooter::Shooter(NoU_Motor* Motor, State* state) : motor(Motor), robotState(state)
{ }

uint8_t Shooter::begin(){
    motor->setInverted(false);

    return 0;
}

int8_t Shooter::update(){
    // safety measure
    if(!robotState->isEnabled()){
        stop();
    }

    // update from action controller
    switch (robotState->getNextAction()){
        case STOP:
            stop();
            break;
        case SUBWOOFER:
            shooterMode = 2;
            setPower = Shooter_SUBWOOFER_kS;
            break;
        case AMP_FORWARD:
            shooterMode = 3;
            setPower = Shooter_AMP_Forward_kS;
            break;
        case AMP_BACKWARD:
            shooterMode = 4;
            setPower = Shooter_AMP_Backward_kS;
            break;
        case PASS:
            shooterMode = 5;
            setPower = Shooter_PASS_kS;
            break;
        case DYNAMIC:
            shooterMode = 6;
            setPower = Shooter_DYNAMIC_kS;
            break;
        case SOURCE:
            shooterMode = 7;
            setPower = Shooter_SOURCE_kS;
            break;
        case -1: // custom speed, do nothing
            break;
    }
    return shooterMode;
}

void Shooter::stop(){
    shooterMode = 0;
    setPower = 0.0;
    execute();
}

void Shooter::execute(){ // actually run the command
    if(robotState->isEnabled()){
        if(motor->getOutput() != setPower){
            motor->set(setPower);
        }
    }
}