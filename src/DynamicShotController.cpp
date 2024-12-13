#include <Arduino.h>
#include "DynamicShotController.h"

DynamicShotController::DynamicShotController(Drivetrain* Drivetrain, Arm* Arm, Shooter* Shooter, PoseEstimator* Pose, State* RobotState)
    : drivetrain(Drivetrain), arm(Arm), shooter(Shooter), pose(Pose), robotState(RobotState)
{ }

uint8_t DynamicShotController::begin()
{

    return 0;
}

uint8_t DynamicShotController::update()
{
    // constantly targeting even if we're not doing dynamic
    if(robotState->getAlliance() == BLUE){
        switch(robotState->getNextAction()){
            case SUBWOOFER:
                currentTargetType = SPEAKER;
                currentTarget = SpeakerBlue;
                break;
            case AMP_FORWARD:
                currentTargetType = AMP;
                currentTarget = AmpBlue;
                break;
            case AMP_BACKWARD:
                currentTargetType = AMP;
                currentTarget = AmpBlue;
                currentTarget.yaw -= 180.0; // flip around given we score reverse
                break;
            case PASS:
                currentTargetType = WING;
                currentTarget = PassBlue;
                break;
            default:
                currentTargetType = NOTARGET;
                currentTarget = Pose();
                break;
        }
    }
    else if (robotState->getAlliance() == RED){
        switch(robotState->getNextAction()){
            case SUBWOOFER:
                currentTargetType = SPEAKER;
                currentTarget = SpeakerRed;
                break;
            case AMP_FORWARD:
                currentTargetType = AMP;
                currentTarget = AmpRed;
                break;
            case AMP_BACKWARD:
                currentTargetType = AMP;
                currentTarget = AmpRed;
                currentTarget.yaw -= 180.0; // flip around given we score reverse
                break;
            case PASS:
                currentTargetType = WING;
                currentTarget = PassRed;
                break;
            default:
                currentTargetType = NOTARGET;
                currentTarget = Pose();
                break;
        }
    }

    Pose currentDeltaPose = pose->subtractPose(pose->getCurrentGlobalPose(), currentTarget);
    float currentDistanceToGoal = pose->lengthOfPose(currentDeltaPose);

    if(currentTargetType == SPEAKER){
        float hypotonuse = sqrt( (deltaHeight*deltaHeight) + (currentDistanceToGoal*currentDistanceToGoal) );

        currentDesiredArmAngle = 90.0 + atan2(armRadius, hypotonuse) + atan2(deltaHeight, currentDistanceToGoal);
    }
    else{ // not speaker
        currentDesiredArmAngle = -1.0;
    }

    currentDesiredTheta = pose->thetaOfPose(currentDeltaPose);

    return 0;
}

void DynamicShotController::execute(){
    if(robotState->isDynamic() && robotState->isEnabled()){
        switch(currentTargetType){
            case SPEAKER:
                arm->setDynamic(currentDesiredArmAngle);
                shooter->execute();
                drivetrain->setTheta(currentDesiredTheta);
                break;
            case AMP:
                arm->execute();
                shooter->execute();
                drivetrain->setTheta(currentTarget.yaw);
                break;
            case PASS:
                arm->execute(); // low pass only
                shooter->execute();
                drivetrain->setTheta(currentDesiredTheta);
                break;
        }
    }
}