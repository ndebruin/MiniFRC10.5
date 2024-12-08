#include <Arduino.h>
#include "Intake.h"

Intake::Intake(NoU_Motor* IntakeMotor, State* state) : intakeMotor(IntakeMotor), robotState(state)
{ }

uint8_t Intake::begin(){
    intakeMotor->setInverted(false);

    pinMode(pinFeedbackLED, OUTPUT);
    pinMode(pinSensor1,INPUT);
    pinMode(pinSensor2,INPUT);

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
            intakeMode = 1;
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
        default:
            break;
    }
    return intakeMode;

    rawSensor1 = analogRead(pinSensor1);
    rawSensor2 = analogRead(pinSensor2);

    if(rawSensor1 > sensor1ValueEmpty || rawSensor2 > sensor2ValueEmpty){
        robotState->setNote(true);
        digitalWrite(pinFeedbackLED, HIGH);
    }
    else{
        robotState->setNote(false);
        digitalWrite(pinFeedbackLED, LOW);
    }
}

void Intake::execute(){
    if(robotState->isEnabled()){
        if(intakeMode == INTAKE && robotState->hasNote()){ // autoStop Intake
            robotState->setNextAction(STOP);
        }
        if(setPower != intakeMotor->getOutput()){
            intakeMotor->set(setPower);
        }
    }
}