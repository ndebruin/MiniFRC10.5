#ifndef POSEESTIMATOR_h
#define POSEESTIMATOR_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include <TinyMatrixMath.hpp>

#include "State.h"

struct Pose{
    double x;
    double y;
    double yaw;
};

class PoseEstimator
{
    public:
        PoseEstimator();

        uint8_t begin();
        uint8_t update();

        double getYaw(){
            return rawYaw - yawOffset;
        }

        void setRawDeltaPos(int64_t PosX, int64_t PosY, uint64_t deltaNS){
            
        }

        void setRawYaw(double Yaw){
            currentGlobalPose.yaw = currentGlobalPose.yaw - yawOffset;
            rawYaw = Yaw;
        }

        void zeroYaw(){
            yawOffset = rawYaw;
        }



    private:
        Pose currentGlobalPose;
        double rawYaw;

        double yawOffset;

        Pose GlobaltoRobotPose(Pose globalPose);
        Pose RobottoGlobalPose(Pose robotPose);

        
 
};

#endif // POSEESTIMATOR_h