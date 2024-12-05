#include <Arduino.h>
#include "PoseEstimator.h"









Pose PoseEstimator::RobottoGlobalPose(Pose robotPose){
    Pose globalPose;

    globalPose.yaw = robotPose.yaw - yawOffset;
    globalPose.x = robotPose.x * cos(globalPose.yaw) + -robotPose.y * sin(globalPose.yaw);
    globalPose.y = robotPose.x * sin(globalPose.yaw) + robotPose.y * cos(globalPose.yaw);

    return globalPose;
}

Pose PoseEstimator::GlobaltoRobotPose(Pose globalPose){
    Pose robotPose;

    robotPose.yaw = globalPose.yaw;
    robotPose.x = globalPose.x * cos(globalPose.yaw) + globalPose.y * sin(globalPose.yaw);
    robotPose.y = -globalPose.x * sin(globalPose.yaw) + globalPose.y * cos(globalPose.yaw);

    return robotPose;
}