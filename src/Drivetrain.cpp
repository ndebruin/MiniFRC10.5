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

    return 0;
}

void Drivetrain::drive(float linearX, float linearY, float angularZ)
{
    // if(fieldOriented)
    // {
            
    //     float temp =  linearY* cos(pose->getYaw()*DEG_TO_RAD) + linearX* sin(pose->getYaw()*DEG_TO_RAD); // this is the systems of equations form of a 2x2 rotation matrix
    //     linearY = -linearY * sin(pose->getYaw()*DEG_TO_RAD) + linearX* cos(pose->getYaw()*DEG_TO_RAD); // this is also the commonly used field oriented drive equations
    //     linearX = temp;
    // }
    float frontLeftPower  = linearY + angularZ + linearX;
    float frontRightPower = linearY - angularZ - linearX;
    float backLeftPower   = linearY + angularZ - (linearX*strafeCompensation);
    float backRightPower  = linearY - angularZ + (linearX*strafeCompensation);
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
