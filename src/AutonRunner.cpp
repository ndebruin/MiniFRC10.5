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
        // updateFunction(); // will run what we were given in setup
        return 1;
    }


    return 0;
}

void AutonRunner::execute(){
    switch (robotState->selectedAuton){
        case OnePlusTaxiSource:
            OnePieceTaxiSource();
            robotState->setTeleopMode();
            break;
        case OnePlusTaxiAmp:
            OnePieceTaxiAmp();
            robotState->setTeleopMode();
            break;
        case TwoAmp:
            // TwoPieceAmp();
            robotState->setTeleopMode();
            break;
        case TwoSource:
            // TwoPieceSource();
            robotState->setTeleopMode();
            break;
    }
}

void AutonRunner::delayWithoutBlocking(int32_t timeMS){
    startTime = millis();
    while((millis() - startTime) < timeMS){
        updateFunction();
        arm->execute();
        shooter->execute();
    }
    startTime = millis();
}

void AutonRunner::OnePieceTaxiSource(){
    // set us up
    // if(robotState->getAlliance() == BLUE){
    //     pose->resetPose(sourceSideSubBlue);
    // }
    // else{
    //     pose->resetPose(sourceSideSubRed);
    // }
    // robotState->setAutoMode();
    robotState->setNextAction(SUBWOOFER);
    shooter->execute();
    arm->execute();
    // wait a second for shooter wheel to spin up
    delayWithoutBlocking(2000);
    intake->execute();
    delayWithoutBlocking(5000); // wait for shoot to happen
    robotState->setNextAction(STOP);
    drivetrain->setFieldOriented(false);
    drivetrain->teleopDrive(0.0, -1.0, 0.0);
    delayWithoutBlocking(2000);
    drivetrain->stop();

    // actually drive
    // if(robotState->getAlliance() == BLUE){
    //     drivetrain->setPose(sourceSpikeNoteBlue);
    // }
    // else{
    //     drivetrain->setPose(sourceSpikeNoteRed);
    // }
    // while(!drivetrain->reachedGoal){
    //     updateFunction();
    // }
    // drivetrain->stop();
}


void AutonRunner::OnePieceTaxiAmp(){
    // set us up
    if(robotState->getAlliance() == BLUE){
        pose->resetPose(ampSideSubBlue);
    }
    else{
        pose->resetPose(ampSideSubRed);
    }
    robotState->setAutoMode();
    robotState->setNextAction(SUBWOOFER);
    shooter->execute();
    arm->execute();
    // wait a second for shooter wheel to spin up
    delayWithoutBlocking(1000);
    intake->execute();
    delayWithoutBlocking(3000); // wait for shoot to happen
    robotState->setNextAction(STOP);

    // actually drive
    if(robotState->getAlliance() == BLUE){
        drivetrain->setPose(ampSpikeNoteBlue);
    }
    else{
        drivetrain->setPose(ampSpikeNoteRed);
    }
    while(!drivetrain->reachedGoal){
        updateFunction();
    }
    drivetrain->stop();
    
}