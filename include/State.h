#ifndef State_h
#define State_h

#include <Arduino.h>

#define TELEOP_MODE false
#define AUTO_MODE true

#define STOP 0
#define INTAKE 1
#define SUBWOOFER 2
#define AMP_FORWARD 3
#define AMP_BACKWARD 4
#define PASS 5
#define DYNAMIC 6
#define SOURCE 7
#define CLIMBERS_UP 8
#define CLIMBERS_DOWN 9

#include "Constants.h"

class State
{
    public:
        // true if has note
        bool hasNote(){
            return note;
        } 
        // false - red 
        // true - blue
        bool getAlliance(){
            return blue;
        }

        bool isEnabled(){
            return enabled;
        }

        // true if has note
        void setNote(bool hasNote){
            note = hasNote;
        }
        // false - red 
        // true - blue
        void setAlliance(bool isBlue){
            blue = isBlue;
        }

        void setEnable(bool Enabled){
            enabled = Enabled;
        }

        // 0 - Stop/Stow
        // 1 - Normal Intake
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        // 8 - Climbers Up
        // 9 - Climbers Down
        uint8_t getNextAction(){
            return nextAction;
        }

        // 0 - Stop/Stow
        // 1 - Normal Intake
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        // 8 - Climbers Up
        // 9 - Climbers Down
        void setNextAction(uint8_t NextAction){
            nextAction = NextAction;
        }

        void setAutoMode(){
            robotMode = AUTO_MODE;
        }

        void setTeleopMode(){
            robotMode = TELEOP_MODE;
        }
        // false for teleop, true for auto
        bool RobotMode(){
            return robotMode;
        }

    private:

        bool enabled = false;
        bool note = false;
        bool blue;

        bool robotMode = false;

        // 0 - Stop/Stow
        // 1 - Normal Intake
        // 2 - Subwoofer
        // 3 - Amp Forward
        // 4 - Amp Backward
        // 5 - Pass
        // 6 - Dynamic Shot
        // 7 - Source Intake
        // 8 - Climbers Up
        // 9 - Climbers Down
        uint8_t nextAction;        
};

#endif