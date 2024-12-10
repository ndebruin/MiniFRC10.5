#include <Arduino.h>
#include "Intake.h"

Intake::Intake(NoU_Motor* IntakeMotor, State* state) : intakeMotor(IntakeMotor), robotState(state)
{ }

uint8_t Intake::begin(){
    intakeMotor->setInverted(false);

    pinMode(pinFeedbackLED, OUTPUT);

    stop();

    return 0;
}

int8_t Intake::update(){
    // safety measure
    if(!robotState->isEnabled()){
        stop();
        intakeMotor->set(0);
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

    rawSensor1 = analogRead(pinSensor1);
    rawSensor2 = analogRead(pinSensor2);

    if(rawSensor1 > sensor1ValueNote || rawSensor2 > sensor2ValueNote){
        if(!estimatedNote){
            estimatedNote = true;
            timerValue = millis();
        }
        if((millis() - timerValue) > debounceTime){
            if(!robotState->hasNote()){
                estimatedNote = true;
                robotState->setNote(NOTE);
                digitalWrite(pinFeedbackLED, HIGH);
                if(intakeMode == INTAKE){ // auto stop
                    robotState->setNextAction(STOP);
                }
            } 
        }
    }
    else{
        if(estimatedNote){
            estimatedNote = false;
            timerValue = millis();
        }
        if((millis() - timerValue) > debounceTime){
            if(robotState->hasNote()){
                estimatedNote = false;
                robotState->setNote(NONOTE);
                digitalWrite(pinFeedbackLED, LOW);
                if(intakeMode == SOURCE){ // auto stop
                    robotState->setNextAction(STOP);
                }
            } 
        }
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