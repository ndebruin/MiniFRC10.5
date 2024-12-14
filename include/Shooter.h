#ifndef Shooter_h
#define Shooter_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "State.h"

#include "Constants.h"

class Shooter
{
    public:
        Shooter(NoU_Motor* Motor, State* state);
        uint8_t begin();
        int8_t update();

        // -1 - Custom Speed
        // 0 - Stop
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        int8_t getMode(){ return shooterMode; }

        void stop(); // only to be used in cases of e-stoppage or similar

        void run(double kSM){
            shooterMode = -1;
            setPower = kSM;
            execute();
        }
        void execute();

    private:
        NoU_Motor* motor;

        State* robotState;

        double setPower;

        // -1 - Custom Speed
        // 0 - Stop
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        int8_t shooterMode;

};

#endif