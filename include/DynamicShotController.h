#ifndef DYNAMICSHOT_h
#define DYNAMICSHOT_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "Constants.h"

#include "Arm.h"
#include "Drivetrain.h"
#include "Shooter.h"
#include "PoseEstimator.h"

class DynamicShotController
{
    public:
        DynamicShotController(Drivetrain* Drivetrain, Arm* Arm, Shooter* Shooter, PoseEstimator* Pose, State* RobotState);

        uint8_t begin();
        uint8_t update();

    private:
        Drivetrain* drivetrain;
        Arm* arm;
        Shooter* shooter;
        PoseEstimator* pose;
        State* robotState;

        float currentDesiredArmAngle;

        Pose blueSpeaker = {goalXBlue, goalYBlue, 180.0};
        Pose redSpeaker = {goalXRed, goalYRed, 0.0};
        
 
};

#endif // DYNAMICSHOT_h