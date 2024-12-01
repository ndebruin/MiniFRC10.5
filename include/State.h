#ifndef State_h
#define State_h

#include <Arduino.h>


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

        // 0 - stop
        // 1 - forward amp
        // 2 - backward amp
        // 3 - subwoofer
        // 4 - passing
        // 5 - intake
        // 6 - dynamic
        uint8_t getNextShot(){
            return nextShot;
        }

        // 0 - stop
        // 1 - forward amp
        // 2 - backward amp
        // 3 - subwoofer
        // 4 - passing
        // 5 - intake
        // 6 - dynamic
        void setNextShot(uint8_t NextShot){
            nextShot = NextShot;
        }

        double getYaw(){
            return yaw-yawOffset;
        }

        void setYaw(double Yaw){
            yaw = Yaw;
        }

        void zeroYaw(){
            yawOffset = yaw;
        }

        void setAuto(){
            inAuto = true;
        }

        void setRobotMode(){
            robotMode = false;
        }
        // false for teleop, true for auto
        bool RobotMode(){
            return robotMode;
        }

    private:

        bool enabled =false;
        bool note;
        bool blue;

        bool robotMode = false;

        // 0 - stop
        // 1 - amp
        // 2 - subwoofer
        // 3 - podium
        // 4 - passing
        // 5 - intake
        uint8_t nextShot;

        double yaw;
        double yawOffset;

        long leftCount;
        long leftOffset;
        long rightCount;
        long rightOffset;

        
};

#endif