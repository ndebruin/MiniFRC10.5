#include <Arduino.h>
#include "PoseEstimator.h"

PoseEstimator::PoseEstimator(HardwareSerial *SerialPort, uint BaudRate, uint8_t rxPin, uint8_t txPin) : serial(SerialPort), baud(BaudRate), tx(txPin), rx(rxPin)
{ }

uint8_t PoseEstimator::begin(){
    serial->begin(baudRate, SERIAL_8N1, rx, tx);

    return 0;
}

uint8_t PoseEstimator::update(){
    // update data from the coproc
    if(updateFromCoProc()){
        rawYaw = rxDataStruct.yaw;
        Pose newRobotPoseData;
        newRobotPoseData.x = rxDataStruct.posX * MOUSE_CONVERSION_FACTOR;
        newRobotPoseData.y = rxDataStruct.posY * MOUSE_CONVERSION_FACTOR;
        newRobotPoseData.yaw = rxDataStruct.yaw;
        supplementPose(&currentGlobalPose, RobottoGlobalPose(newRobotPoseData));
        return 1; // got an update
    }
    return 0; // didn't get an update
}

Pose PoseEstimator::RobottoGlobalPose(Pose robotPose){
    Pose globalPose;

    globalPose.yaw = robotPose.yaw - yawOffset;
    globalPose.x = robotPose.x * cos(robotPose.yaw) + -robotPose.y * sin(robotPose.yaw); // this is the systems of equations form of a 2x2 rotation matrix
    globalPose.y = robotPose.x * sin(robotPose.yaw) + robotPose.y * cos(robotPose.yaw); // this is also the transpose (which is equivalent to the inverse in this case) of the commonly used field oriented drive equations

    return globalPose;
}

Pose PoseEstimator::GlobaltoRobotPose(Pose globalPose){
    Pose robotPose;

    robotPose.yaw = globalPose.yaw;
    robotPose.x = globalPose.x * cos(globalPose.yaw) + globalPose.y * sin(globalPose.yaw); // this is the systems of equations form of a 2x2 rotation matrix
    robotPose.y = -globalPose.x * sin(globalPose.yaw) + globalPose.y * cos(globalPose.yaw); // this is also the commonly used field oriented drive equations

    return robotPose;
}

void PoseEstimator::supplementPose(Pose *mainPose, Pose addPose){
    mainPose->yaw = addPose.yaw;
    mainPose->x += addPose.x;
    mainPose->y += addPose.y;
}

Pose PoseEstimator::subtractPose(Pose pose1, Pose pose2){
    Pose newPose;
    newPose.x = pose1.x - pose2.x;
    newPose.y = pose1.y - pose2.y;
    newPose.yaw = pose1.yaw - pose2.yaw;
}

float PoseEstimator::lengthOfPose(Pose pose){
    return sqrt((pose.x*pose.x) + (pose.y*pose.y));
}

bool PoseEstimator::updateFromCoProc()
{   
    // reset rxDataStruct
    rxDataStruct.posX = 0;
    rxDataStruct.posY = 0;
    rxDataStruct.yaw = 0;
    
    if(serial->available())
    {
        serial->readStringUntil('b'); // start of packet
        rxDataStruct.posX = serial->readStringUntil('x').toInt();
        rxDataStruct.posY = serial->readStringUntil('y').toInt();
        rxDataStruct.yaw = serial->readStringUntil('t').toFloat();
        rawYaw = rxDataStruct.yaw;
        return true;
    }
    return false; // didn't update
}