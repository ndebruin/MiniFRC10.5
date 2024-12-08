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
#include "State.h"
#include "PoseEstimator.h"
#include "PathFollower.h"
#include "DynamicShooter.h"

#include "Constants.h"

////////////////////////////////////////////////////////////////////// Hardware Declarations //////////////////////////////////////////////////////////////////////

BluetoothSerial SerialBluetooth; // bluetooth link

HardwareSerial coProcSerial(2);

PoseEstimator pose(&coProcSerial, baudRate, RXPin, TXPin);

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

Drivetrain drivetrain = Drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &pose, &state);

Arm arm = Arm(&armServo, &state); 
Shooter shooter = Shooter(&shooterMotor, &state);

Intake intake = Intake(&intakeMotor, &state);

// create our object definitions for fancy stuff (path following, autoaim, etc)

// PathFollower pathFollower = PathFollower(&drivetrain, &pose, &state);

// DynamicShooter shooterAim = DynamicShooter(&drivetrain, &arm, &shooter, &pose, &state);

////////////////////////////////////////////////////////////////////// Function Declerations //////////////////////////////////////////////////////////////////////

void asyncUpdate();

double deadzone(double raw);

void runDrivetrain();
void runStateSelector(); // shot presets, climber, and intake

////////////////////////////////////////////////////////////////////// Global Variables //////////////////////////////////////////////////////////////////////

bool justExecuted = false;
bool justIntake = false;

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  // start up bluetooth link for alfredoconnect
  SerialBluetooth.begin(robotName);
  AlfredoConnect.begin(SerialBluetooth);

  // start RSL
  RSL::initialize();

  // start subsystems
  drivetrain.begin();
  arm.begin();
  shooter.begin();
  intake.begin();
  pinMode(pinSensor1, INPUT);

  // start our pose estimator
  pose.begin();
  
  // start advanced controllers
  // pathFollower.begin();
  // shooterAim.begin();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

void loop() 
{
  asyncUpdate(); // updates all the things that need to be updated every loop regardless of anything else

  SerialBluetooth.println("sensor1:" + String(intake.sensor1Value()) + " sensor2:" + String(intake.sensor2Value()));

  // SerialBluetooth.println(pose.getYaw());
  // SerialBluetooth.println(state.getNextAction());

  if(state.RobotMode() == TELEOP_MODE){ // if in teleop
    // handle drivetrain
    runDrivetrain();
    // handle state machine decisions    
    runStateSelector();

    if(AlfredoConnect.buttonHeld(0, buttonZeroYaw)){ // reset IMU yaw
      pose.zeroYaw();
    }
    if(AlfredoConnect.buttonHeld(0, buttonEnableFieldOriented)){ // enable / disable field oriented driving
      drivetrain.setDriveMode(FIELD_ORIENTED);
    }
    else if(AlfredoConnect.buttonHeld(0, buttonDisableFieldOriented)){
      drivetrain.setDriveMode(ROBOT_ORIENTED);
    }
  }
}

////////////////////////////////////////////////////////////////////// Function Definitions //////////////////////////////////////////////////////////////////////

void asyncUpdate(){
  // let subsystems code update
  drivetrain.update();
  intake.update();
  arm.update();
  shooter.update();

  // update our pose
  pose.update();

  // let advanced controllers update
  // shooterAim.update();
  // pathFollower.update();

  // update from driver station
  AlfredoConnect.update();

  // rsl code
  RSL::update();

  if(state.isEnabled()){
    RSL::setState(RSL_ENABLED);
  }
  else if(!state.isEnabled()){
    RSL::setState(RSL_OFF);
  }
}

double deadzone(double rawJoy){
  if(fabs(rawJoy) < deadzoneValue){
    return 0.0;
  }
  return rawJoy;
}

void runDrivetrain(){
  float linearX = deadzone(AlfredoConnect.getAxis(0, axisLinX));
  float linearY = -deadzone(AlfredoConnect.getAxis(0, axisLinY));
  float angularZ = deadzone(AlfredoConnect.getAxis(0, axisAngZ));
  
  drivetrain.drive(linearX, linearY, angularZ);

  return;
}

void runStateSelector(){
  // this giant mess of if statements handles all the state transitions
  if(AlfredoConnect.buttonHeld(0, buttonIntake) && state.getNextAction() <= INTAKE && !state.hasNote()){ // we want to only go to an intake state if we're not currently doing something else
    state.setNextAction(INTAKE);
    justIntake = true;
  }
  else if(AlfredoConnect.buttonHeld(0, buttonSource)){ // manual "reverse"
    state.setNextAction(SOURCE);
    justIntake = true;
  }
  else if(justIntake && 
          (!AlfredoConnect.buttonHeld(0, buttonIntake) && state.getNextAction() <= INTAKE) || 
          (!AlfredoConnect.buttonHeld(0, buttonSource) && state.getNextAction() == SOURCE)){ // intake should stop if we let go of the button for source or ground
    state.setNextAction(STOP);
    justIntake = false;
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
  else if(AlfredoConnect.buttonHeld(0, buttonAmpBackward)){
    state.setNextAction(AMP_BACKWARD);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonDynamic)){
    state.setNextAction(DYNAMIC);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonClimbUp)){
    state.setNextAction(CLIMBERS_UP);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonClimbDown)){
    state.setNextAction(CLIMBERS_DOWN);
  }

  // we only want to do this if we're currently in another state
  // (as in, if we are trying to shoot)
  if(AlfredoConnect.buttonHeld(0, buttonIntake) && state.getNextAction() > INTAKE){ 
    intake.execute();
  }



  // actually have the execute button do it's thing
  if(AlfredoConnect.buttonHeld(0, buttonExecute)){
    arm.execute();
    shooter.execute();
    justExecuted = true;
  }
  if(justExecuted && !AlfredoConnect.buttonHeld(0, buttonExecute) && state.getNextAction() > INTAKE){
    state.setNextAction(STOP);
    justExecuted = false;
  }

  // enable / disable logic
  if(AlfredoConnect.buttonHeld(0, buttonEnable)){
    state.setEnable(true);
  }
  else if(AlfredoConnect.buttonHeld(0, buttonDisable)){
    state.setEnable(false);
  }
}

