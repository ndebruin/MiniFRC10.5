#ifndef Drivetrain_h
#define Drivetrain_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "PoseEstimator.h"


#include "State.h"

#define FIELD_ORIENTED true
#define ROBOT_ORIENTED false

class Drivetrain
{
    public:
        Drivetrain(NoU_Motor* FrontLeftMotor, NoU_Motor* FrontRightMotor, NoU_Motor* BackLeftMotor, NoU_Motor* BackRightMotor, PoseEstimator* poseEstimator, State* robotState);
        uint8_t begin();
        uint8_t update();

        bool getDriveMode(){ return fieldOriented; }

        void setDriveMode(bool driveMode){ fieldOriented = driveMode; }

        // will either be field oriented or not depending on the seperately set field (setFieldOriented(bool))
        void drive(float linearX, float linearY, float angularZ);

        // will either be field oriented or not depending on the passed boolean
        void drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled); 

        void stop();

    private:
        NoU_Motor* frontLeftMotor;
        NoU_Motor* frontRightMotor;
        NoU_Motor* backLeftMotor;
        NoU_Motor* backRightMotor;

        PoseEstimator* pose;
        State* robotState;

        bool fieldOriented = false;
 
};

#endif