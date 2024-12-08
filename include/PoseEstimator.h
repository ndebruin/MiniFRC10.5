#ifndef POSEESTIMATOR_h
#define POSEESTIMATOR_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "coProcCom.h"
#include "Constants.h"

struct Pose{
    double x;
    double y;
    double yaw;
};

class PoseEstimator
{
    public:
        PoseEstimator(HardwareSerial *SerialPort, uint BaudRate, uint8_t rxPin, uint8_t txPin);

        uint8_t begin();
        uint8_t update();

        double getYaw(){
            return currentGlobalPose.yaw;
        }

        void zeroYaw(){
            yawOffset = rawYaw;
        }

        Pose GlobaltoRobotPose(Pose globalPose);
        Pose RobottoGlobalPose(Pose robotPose);

    private:
        HardwareSerial *serial;
        uint baud;
        uint8_t tx, rx;

        CoProcStructRX rxDataStruct;

        Pose currentGlobalPose;
        double rawYaw;

        double yawOffset;
        
        void supplementPose(Pose *mainPose, Pose addPose);

        bool coProcReceive(CoProcStructRX* table);
        bool updateFromCoProc();
        
 
};

#endif // POSEESTIMATOR_h