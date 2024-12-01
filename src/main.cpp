#include <Arduino.h>

// Alfredo Stuff
#include <Alfredo_NoU2.h>
#include <BluetoothSerial.h>
#include <AlfredoConnect.h>

// Subsystems
#include "Arm.h"
#include "Intake.h"
#include "Shooter.h"
#include "Drivetrain.h"

// other custom code
#include "coProcCom.h"
#include "State.h"
#include "PoseEstimator.h"
#include "PathFollower.h"
#include "DynamicShooter.h"

#include "Constants.h"

////////////////////////////////////////////////////////////////////// Hardware Declarations //////////////////////////////////////////////////////////////////////

BluetoothSerial SerialBluetooth; // bluetooth link


PoseEstimator robotPose;

State state;

// create our actual motors and servos

NoU_Motor frontLeftMotor(frontLeftMotorChannel);
NoU_Motor frontRightMotor(frontRightMotorChannel);
NoU_Motor backLeftMotor(backLeftMotorChannel);
NoU_Motor backRightMotor(backRightMotorChannel);

NoU_Servo armServo(armServoChannel);
NoU_Motor intakeMotor(intakeMotorChannel);
NoU_Motor intakeMotor(intakeMotorChannel);


// create our subsystem objects

Drivetrain drivetrain = Drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &robotPose, &state);

Arm arm = Arm(&armServo, &state);

Shooter shooter = Shooter(&shooterMotor, &state);

Intake intake = Intake(&intakeMotor, &state);


// create our object definitions for fancy stuff (path following, autoaim, etc)

PathFollower pathFollower = PathFollower(&drivetrain, &robotPose, &state);

DynamicShooter shooterAim = DynamicShooter(&drivetrain, &arm, &shooter, &robotPose, &state);


void setup() 
{
  Serial.begin(9600); // start coprocesser connection

  // start up bluetooth link for alfredoconnect
  SerialBluetooth.begin(robotName);
  AlfredoConnect.begin(SerialBluetooth);

  // start RSL
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  drivetrain.begin();
  arm.begin();
  shooter.begin();
  intake.begin();

  robotPose.begin();
  
  pathFollower.begin();
  shooterAim.begin();
}

bool updateCoProc = false;

CoProcStructTX txDataStruct;
CoProcStructRX rxDataStruct;

void loop() 
{
  // update data from the coproc
  if(updateFromCoProc(&rxDataStruct)){
    robotPose.setRawPos(rxDataStruct.posX, rxDataStruct.posY);
    robotPose.setRawYaw(rxDataStruct.yaw);
    shooterAim.setHaveTarget(rxDataStruct.camTargetDetected);
    shooterAim.setRawPose(rxDataStruct.camX, rxDataStruct.camY);
  }

  // update from driver station
  AlfredoConnect.update();

  // if we need to send data to the coProc
  if(updateCoProc){
    updateCoProc = false;
    coProcSend(&txDataStruct);
  }

  if(state.RobotMode()){ // if in teleop
    float linearX = AlfredoConnect.getAxis(0, axisLinX);
    float linearY = AlfredoConnect.getAxis(0, axisLinY);
    float angularZ = AlfredoConnect.getAxis(0, axisAngZ);

    drivetrain.drive(linearX, linearY, angularZ);
  }

// example code for autoaim
  // if(autoaim){
  //   txDataStruct.camFlash = true; // enable the flashlight on the ESP32Cam
  //   updateCoProc = true;
  //   shooterAim.enable();
  // }
  // else{
  //   txDataStruct.camFlash = false;
  //   updateCoProc = true;
  //   shooterAim.disable();
  // }
}

