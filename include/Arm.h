#ifndef ARM_h
#define ARM_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "State.h"

#include "Constants.h"

class Arm
{
    public:
        Arm(NoU_Servo* ArmServo, State* state);
        uint8_t begin();
        int8_t update();

        // -1 - Custom Angle
        // 0 - Stop/Stow
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        // 8 - Climbers Up
        // 9 - Climbers Down
        int8_t getMode(){ return armMode; }

        void execute();

        void set(double angle){
            armMode = -1;
            armSetAngle = angle;
        }

        void setDynamic(double angle);


    private:
        NoU_Servo* armServo;

        State* robotState;

        double armSetAngle;

        // -1 - Custom Angle
        // 0 - Stop/Stow
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        // 8 - Climbers Up
        // 9 - Climbers Down
        int8_t armMode;

        void home();

};

#endif