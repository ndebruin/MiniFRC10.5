#ifndef Intake_h
#define Intake_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "State.h"

#include "Constants.h"

class Intake
{
    public:
        Intake(NoU_Motor* IntakeMotor, State* state);
        uint8_t begin();
        int8_t update();

        void execute();

        // -1 - Custom Speed
        // 0 - Stop
        // 1 - Intake
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        int8_t getMode(){ return intakeMode; }

        void set(double kS){
            intakeMode = -1;
            setPower = kS;
            execute();
        }

        void setRawSensor(uint16_t sensor1, uint16_t sensor2);

        void stop(){
            intakeMode = 0;
            setPower = 0.0;
            execute();
        }

        uint16_t sensor1Value() {return rawSensor1;}
        uint16_t sensor2Value() {return rawSensor2;}

    private:
        NoU_Motor* intakeMotor;

        State* robotState;

        uint16_t rawSensor1;
        uint16_t rawSensor2;

        double setPower;

        // -1 - Custom Speed
        // 0 - Stop
        // 1 - Intake
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        int8_t intakeMode;


};

#endif