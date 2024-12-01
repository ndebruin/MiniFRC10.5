#include <Arduino.h>
#include "Drivetrain.h"

Drivetrain(NoU_Motor* FrontLeftMotor, NoU_Motor* FrontRightMotor, NoU_Motor* BackLeftMotor, NoU_Motor* BackRightMotor, PoseEstimator* poseEstimator)
                                    : frontLeftMotor(FrontLeftMotor), frontRightMotor(FrontRightMotor), backLeftMotor(BackLeftMotor), backRightMotor(BackRightMotor), pose(poseEstimator)
{ }

bool Drivetrain::isFieldOriented(){ return fieldOriented; }

void Drivetrain::setFieldOriented(bool fieldOrientedEnabled)
{
    fieldOriented = fieldOrientedEnabled;
}

uint8_t Drivetrain::begin()
{
    frontLeftMotor->setInverted(false);
    frontRightMotor->setInverted(true);
    backLeftMotor->setInverted(false);
    backRightMotor->setInverted(true);

    return 0;
    
}

void Drivetrain::drive(float linearX, float linearY, float angularZ)
{
    if(fieldOriented)
    {
        float temp = linearX * cos(pose->getYaw()) + linearY * sin(pose->getYaw());
        linearY = -linearX * sin(pose->getYaw()) + linearY * cos(pose->getYaw());
        linearX = temp;
    }
    float frontLeftPower = linearX + linearY + angularZ;
    float frontRightPower = -linearX + linearY - angularZ;
    float rearLeftPower = -linearX + linearY + angularZ;
    float rearRightPower = linearX + linearY - angularZ;
    float maxMagnitude = max(fabs(frontLeftPower), max(fabs(frontRightPower), max(fabs(rearLeftPower), fabs(rearRightPower))));
    if (maxMagnitude > 1) 
    {
        frontLeftPower /= maxMagnitude;
        frontRightPower /= maxMagnitude;
        rearLeftPower /= maxMagnitude;
        rearRightPower /= maxMagnitude;
    }
    if(state->isEnabled())
    {
        frontLeftMotor.write(frontLeftPower);
        frontRightMotor.write(frontRightPower);
        backLeftMotor.write(backLeftPower);
        backRightMotor.write(backRightPower);
    }

    return;
}

void Drivetrain::drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled)
{
    fieldOriented = fieldOrientedEnabled;
    drive(linearX, linearY, angularZ);

    return;
}
