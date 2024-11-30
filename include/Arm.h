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
        uint8_t update();

        // 0 - Stow
        // 1 - Custom Angle
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Climb Up
        // 7 - Climb Down
        // 8 - Dynamic Shot
        uint8_t getMode();

        void setMode(uint8_t setMode);

        void set(double angle);

        void setDynamic(double angle);


    private:
        NoU_Servo* armServo;

        State* robotState;

        double armSetAngle;

        // 0 - Stow
        // 1 - Custom Angle
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Climb Up
        // 7 - Climb Down
        // 8 - Dynamic Shot
        uint8_t armMode;

};

#endif