#ifndef DYNAMICSHOT_h
#define DYNAMICSHOT_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "Constants.h"

#include "Arm.h"
#include "Drivetrain.h"
#include "Shooter.h"
#include "PoseEstimator.h"
#include "FieldDims.h"

#define NOTARGET 0
#define SPEAKER 1
#define AMP 2
#define WING 3

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

        Pose currentTarget;

        float currentDesiredArmAngle;
        float currentDesiredTheta;

        uint8_t currentTargetType;
 
};

#endif // DYNAMICSHOT_h