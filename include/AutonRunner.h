#ifndef AUTONRUNNER_h
#define AUTONRUNNER_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "Constants.h"

#include "Arm.h"
#include "Drivetrain.h"
#include "Shooter.h"
#include "Intake.h"
#include "PoseEstimator.h"
#include "State.h"
#include "FieldDims.h"

#include "AutonDefs.h"

#define OnePlusTaxiSource 1
#define OnePlusTaxiAmp 2
#define TwoCenter 2
#define TwoSource 3
#define TwoAmp 4

typedef void (*UpdateFunction)();

class AutonRunner
{
    public:
        AutonRunner(Drivetrain* Drivetrain, Arm* Arm, Shooter* Shooter, Intake* Intake, PoseEstimator* Pose, State* RobotState);

        uint8_t begin(UpdateFunction backgroundFunction);
        uint8_t update();
        void execute();

    private:
        void delayWithoutBlocking(int32_t millis);
        uint32_t startTime;

        void OnePieceTaxiSource();
        void OnePieceTaxiAmp();
        void TwoPieceCenter();
        void TwoPieceSource();
        void TwoPieceAmp();

        UpdateFunction updateFunction;
        Drivetrain* drivetrain;
        Arm* arm;
        Shooter* shooter;
        Intake* intake;
        PoseEstimator* pose;
        State* robotState;
};

#endif // AUTONRUNNER_h