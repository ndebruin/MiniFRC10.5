#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include "Drivetrain.h"

Drivetrain::Drivetrain(NoU_Motor* FrontLeftMotor, NoU_Motor* FrontRightMotor, NoU_Motor* BackLeftMotor, NoU_Motor* BackRightMotor, PoseEstimator* poseEstimator, State* RobotState)
                                    : frontLeftMotor(FrontLeftMotor), frontRightMotor(FrontRightMotor), backLeftMotor(BackLeftMotor), backRightMotor(BackRightMotor), pose(poseEstimator), robotState(RobotState)
{ }



uint8_t Drivetrain::begin()
{
    frontLeftMotor->setInverted(false);
    frontRightMotor->setInverted(true);
    backLeftMotor->setInverted(false);
    backRightMotor->setInverted(true);

    return 0;
    
}

uint8_t Drivetrain::update(){
    // safety measure
    if(!robotState->isEnabled()){
        stop();
    }
}

void Drivetrain::drive(float linearX, float linearY, float angularZ)
{
    Pose desiredCommand;
    if(fieldOriented)
    {
        desiredCommand = pose->GlobaltoRobotPose(desiredCommand);
    }
    float frontLeftPower = desiredCommand.x + desiredCommand.y + desiredCommand.yaw;
    float frontRightPower = -desiredCommand.x + desiredCommand.y - desiredCommand.yaw;
    float backLeftPower = -desiredCommand.x + desiredCommand.y + desiredCommand.yaw;
    float backRightPower = desiredCommand.x + desiredCommand.y - desiredCommand.yaw;
    float maxMagnitude = max(fabs(frontLeftPower), max(fabs(frontRightPower), max(fabs(backLeftPower), fabs(backRightPower))));
    if (maxMagnitude > 1) 
    {
        frontLeftPower /= maxMagnitude;
        frontRightPower /= maxMagnitude;
        backLeftPower /= maxMagnitude;
        backRightPower /= maxMagnitude;
    }
    if(robotState->isEnabled())
    {
        frontLeftMotor->set(frontLeftPower);
        frontRightMotor->set(frontRightPower);
        backLeftMotor->set(backLeftPower);
        backRightMotor->set(backRightPower);
    }

    return;
}

void Drivetrain::drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled)
{
    fieldOriented = fieldOrientedEnabled;
    drive(linearX, linearY, angularZ);

    return;
}

void Drivetrain::stop(){
    frontLeftMotor->set(0);
    frontRightMotor->set(0);
    backLeftMotor->set(0);
    backRightMotor->set(0);

    return;
}
