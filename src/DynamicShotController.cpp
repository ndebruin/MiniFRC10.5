#include <Arduino.h>
#include "DynamicShotController.h"

DynamicShotController::DynamicShotController(Drivetrain* Drivetrain, Arm* Arm, Shooter* Shooter, PoseEstimator* Pose, State* RobotState)
    : drivetrain(Drivetrain), arm(Arm), shooter(Shooter), pose(Pose), robotState(RobotState)
{ }

uint8_t DynamicShotController::begin()
{

}

uint8_t DynamicShotController::update()
{
    float currentDistanceToGoal;
    if(robotState->getAlliance() == BLUE){
        currentDistanceToGoal = pose->lengthOfPose(pose->subtractPose(pose->getCurrentGlobalPose(), blueSpeaker));
    }
    else if(robotState->getAlliance() == RED){
        currentDistanceToGoal = pose->lengthOfPose(pose->subtractPose(pose->getCurrentGlobalPose(), redSpeaker));
    }
    
    float hypotonuse = sqrt( (deltaHeight*deltaHeight) + (currentDistanceToGoal*currentDistanceToGoal) );

    currentDesiredArmAngle = 90.0 + atan2(armRadius, hypotonuse) + atan2(deltaHeight, currentDistanceToGoal);
}