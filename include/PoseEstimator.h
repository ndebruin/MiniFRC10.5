#ifndef POSEESTIMATOR_h
#define POSEESTIMATOR_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "State.h"

class PoseEstimator
{
    public:
        PoseEstimator();

        uint8_t begin();
        uint8_t update();

        double getYaw(){
            return rawYaw - yawOffset;
        }

        void setRawPos(int64_t PosX, int64_t PosY){
            rawPosX = PosX;
            rawPosY = PosY;
        }
        void setRawYaw(double Yaw){
            rawYaw = Yaw;
        }

    private:

        double rawYaw;

        double yawOffset;

        int64_t rawPosX;
        int64_t rawPosY;

        
 
};

#endif // POSEESTIMATOR_h