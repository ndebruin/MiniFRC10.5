#include <Arduino.h>
#include "Intake.h"

Intake::Intake(NoU_Motor* IntakeMotor, State* state) : intakeMotor(IntakeMotor), robotState(state)
{ }

uint8_t Intake::begin(){
    intakeMotor->setInverted(true);

    stop();

    return 0;
}

int8_t Intake::update(){
    // safety measure
    if(!robotState->isEnabled()){
        stop();
    }

    // update from action controller
    switch (robotState->getNextAction()){
        case STOP:
            stop();
            break;
        case INTAKE:
            intakeMode = 2;
            setPower = Intake_IN_kS;
            execute();
            break;
        case SUBWOOFER:
            intakeMode = 2;
            setPower = Intake_SHOOT_kS;
            break;
        case AMP_FORWARD:
            intakeMode = 3;
            setPower = Intake_AMP_kS;
            break;
        case AMP_BACKWARD:
            intakeMode = 4;
            setPower = Intake_SHOOT_kS;
            break;
        case PASS:
            intakeMode = 5;
            setPower = Intake_SHOOT_kS;
            break;
        case DYNAMIC:
            intakeMode = 6;
            setPower = Intake_SHOOT_kS;
            break;
        case SOURCE:
            intakeMode = 7;
            setPower = Intake_REVERSE_IN_kS;
            execute();
            break;
        case CLIMBERS_UP:
            stop();
            break;
        case CLIMBERS_DOWN:
            stop();
            break;
        case -1: // custom speed, do nothing
            break;
        default:
            break;
    }
    return intakeMode;
}

void Intake::execute(){
    if(robotState->isEnabled()){
        if(setPower != intakeMotor->getOutput()){
            intakeMotor->set(setPower);
        }
    }
}