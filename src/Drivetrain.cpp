#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include "Drivetrain.h"

Drivetrain::Drivetrain(NoU_Drivetrain* NoUDrivetrain, PoseEstimator* poseEstimator, State* RobotState) : nouDrivetrain(NoUDrivetrain), pose(poseEstimator), robotState(RobotState)
{ }



uint8_t Drivetrain::begin()
{
    // setup intake curve values
    nouDrivetrain->setMinimumOutput(kV);
    nouDrivetrain->setInputExponent(driveExp);

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
    // float frontLeftPower  = linearY + angularZ + linearX;
    // float frontRightPower = linearY - angularZ - linearX;
    // float backLeftPower   = linearY + angularZ - (linearX*strafeCompensation);
    // float backRightPower  = linearY - angularZ + (linearX*strafeCompensation);
    // float maxMagnitude = max(fabs(frontLeftPower), max(fabs(frontRightPower), max(fabs(backLeftPower), fabs(backRightPower))));
    // if (maxMagnitude > 1) 
    // {
    //     frontLeftPower /= maxMagnitude;
    //     frontRightPower /= maxMagnitude;
    //     backLeftPower /= maxMagnitude;
    //     backRightPower /= maxMagnitude;
    // }
    if(robotState->isEnabled())
    {
        // frontLeftMotor->set(frontLeftPower);
        // frontRightMotor->set(frontRightPower);
        // backLeftMotor->set(backLeftPower);
        // backRightMotor->set(backRightPower);
        nouDrivetrain->holonomicDrive(linearX, linearY, angularZ);
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
    nouDrivetrain->holonomicDrive(0, 0, 0);

    return;
}
