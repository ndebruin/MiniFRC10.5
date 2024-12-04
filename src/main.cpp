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
NoU_Motor shooterMotor(shooterMotorChannel);


// create our subsystem objects

Drivetrain drivetrain = Drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &robotPose, &state);

Arm arm = Arm(&armServo, &state);

Shooter shooter = Shooter(&shooterMotor, &state);

Intake intake = Intake(&intakeMotor, &state);


// create our object definitions for fancy stuff (path following, autoaim, etc)

// PathFollower pathFollower = PathFollower(&drivetrain, &robotPose, &state);

// DynamicShooter shooterAim = DynamicShooter(&drivetrain, &arm, &shooter, &robotPose, &state);

////////////////////////////////////////////////////////////////////// Function Declerations //////////////////////////////////////////////////////////////////////

double deadzone(double raw);

void runDrivetrain();
void runStateSelector(); // shot presets, climber, and intake

////////////////////////////////////////////////////////////////////// Global Variables //////////////////////////////////////////////////////////////////////

bool updateCoProc = false;

CoProcStructTX txDataStruct;
CoProcStructRX rxDataStruct;

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

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
  
  // pathFollower.begin();
  // shooterAim.begin();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

void loop() 
{
  // update data from the coproc
  if(updateFromCoProc(&rxDataStruct)){
    robotPose.setRawPos(rxDataStruct.posX, rxDataStruct.posY);
    robotPose.setRawYaw(rxDataStruct.yaw);
    // shooterAim.setHaveTarget(rxDataStruct.camTargetDetected);
    // shooterAim.setRawPos(rxDataStruct.camX, rxDataStruct.camY);
  }

  // update from driver station
  AlfredoConnect.update();

  // if we need to send data to the coProc
  if(updateCoProc){
    updateCoProc = false;
    coProcSend(&txDataStruct);
  }

  if(state.RobotMode() == TELEOP_MODE){ // if in teleop
    // handle drivetrain
    runDrivetrain();
    // handle state machine decisions    
    runStateSelector();
  }

  // let subsystems code update
  drivetrain.update();
  intake.update();
  arm.update();
  shooter.update();

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

////////////////////////////////////////////////////////////////////// Function Definitions //////////////////////////////////////////////////////////////////////

double deadzone(double rawJoy){
  if(fabs(rawJoy) < deadzoneValue){
    return 0.0;
  }
  return rawJoy;
}

void runDrivetrain(){
  float linearX = deadzone(AlfredoConnect.getAxis(0, axisLinX));
  float linearY = deadzone(AlfredoConnect.getAxis(0, axisLinY));
  float angularZ = deadzone(AlfredoConnect.getAxis(0, axisAngZ));
  
  drivetrain.drive(linearX, linearY, angularZ);

  return;
}

void runStateSelector(){
  // this giant mess of if statements handles all the state transitions
  if(AlfredoConnect.buttonHeld(0, buttonIntake) && state.getNextAction() < 2){ // we want to only go to an intake state if we're not currently doing something else
    state.setNextAction(INTAKE);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonSub)){
    state.setNextAction(SUBWOOFER);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonAmpForward)){
    state.setNextAction(AMP_FORWARD);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonPass)){
    state.setNextAction(PASS);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonSource)){
    state.setNextAction(SOURCE);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonDynamic)){
    state.setNextAction(DYNAMIC);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonClimb)){
    state.setNextAction(CLIMBERS_UP);
  }

  // we only want to do this if we're currently in another state
  // (as in, if we are trying to shoot)
  if(AlfredoConnect.buttonHeld(0, buttonIntake) && state.getNextAction() > 1){ 
    intake.execute();
  }

  // actually have the execute button do it's thing
  if(AlfredoConnect.buttonHeld(0, buttonExecute)){
    arm.execute();
    shooter.execute();
  }

  // if we were previously in the climbers up state, but are no longer holding the button, we want to transition into the climbers down state
  // (which is not the arm stow state)
  // we can transition out of it if needed
  if(state.getNextAction() == CLIMBERS_UP && !AlfredoConnect.buttonHeld(0, buttonClimb)){ 
    state.setNextAction(CLIMBERS_DOWN);
  }
}

