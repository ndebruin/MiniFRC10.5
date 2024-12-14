#include <Arduino.h>
#include "AutonRunner.h"

AutonRunner::AutonRunner(Drivetrain* Drivetrain, Arm* Arm, Shooter* Shooter, Intake* Intake, PoseEstimator* Pose, State* RobotState):drivetrain(Drivetrain), arm(Arm), shooter(Shooter), intake(Intake), pose(Pose), robotState(RobotState)
{ }


uint8_t AutonRunner::begin(UpdateFunction backgroundFunction){
    updateFunction = backgroundFunction;

    return 0;
}


uint8_t AutonRunner::update(){
    if(robotState->getRobotMode() == AUTO_MODE){
        updateFunction(); // will run what we were given in setup
        return 1;
    }


    return 0;
}

void AutonRunner::delayWithoutBlocking(uint32_t time){
    startTime = millis();
    while((millis() - startTime) < time){
        updateFunction();
    }
    startTime = millis();
}