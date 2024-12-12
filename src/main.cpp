#include <Arduino.h>

// Alfredo Stuff
#include <Alfredo_NoU2.h>
// #include <BluetoothSerial.h>
// #include <AlfredoConnect.h>
#include <PestoLink-Receive.h>

// Subsystems
#include "Arm.h"
#include "Intake.h"
#include "Shooter.h"
#include "Drivetrain.h"

// other custom code
#include "State.h"
#include "PoseEstimator.h"
#include "PathFollower.h"
#include "DynamicShotController.h"

#include "Constants.h"

////////////////////////////////////////////////////////////////////// Hardware Declarations //////////////////////////////////////////////////////////////////////

// BluetoothSerial SerialBluetooth; // bluetooth link

PoseEstimator pose(&Serial, baudRate); // we're just running the coproc serial over the normal serial bus.
                                       // this kinda sucks bc it means that we need to disconnect it whenever we program, but oh well
                                       // can still use Serial.println() for normal computer info tho

State state;

// create our actual motors and servos

NoU_Motor frontLeftMotor(frontLeftMotorChannel);
NoU_Motor frontRightMotor(frontRightMotorChannel);
NoU_Motor backLeftMotor(backLeftMotorChannel);
NoU_Motor backRightMotor(backRightMotorChannel);

NoU_Drivetrain nou_drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

NoU_Servo armServo(armServoChannel);
NoU_Motor intakeMotor(intakeMotorChannel);
NoU_Motor shooterMotor(shooterMotorChannel);

// create our subsystem objects

Drivetrain drivetrain = Drivetrain(&nou_drivetrain, &pose, &state);

Arm arm = Arm(&armServo, &state); 
Shooter shooter = Shooter(&shooterMotor, &state);

Intake intake = Intake(&intakeMotor, &state);

// create our object definitions for fancy stuff (path following, autoaim, etc)

// PathFollower pathFollower = PathFollower(&drivetrain, &pose, &state);

DynamicShotController shooterAim = DynamicShotController(&drivetrain, &arm, &shooter, &pose, &state);

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
  // SerialBluetooth.begin(robotName);
  // AlfredoConnect.begin(SerialBluetooth);

  // Serial.begin(9600);

  frontRightMotor.setInverted(true);
  backRightMotor.setInverted(true);

  PestoLink.begin(robotName);

  // start RSL
  RSL::initialize();

  // start subsystems
  drivetrain.begin();
  arm.begin();
  shooter.begin();
  intake.begin();

  // start our pose estimator
  pose.begin();
  
  // start advanced controllers
  // pathFollower.begin();
  shooterAim.begin();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

void loop() 
{
  asyncUpdate(); // updates all the things that need to be updated every loop regardless of anything else

  Serial.println("sensor1:" + String(intake.sensor1Value()) + " sensor2:" + String(intake.sensor2Value()));

  // Serial.println(String(pose.getCurrentGlobalPose().x) + "x" + String(pose.getCurrentGlobalPose().y) + "y" + String(pose.getCurrentGlobalPose().yaw) + "t");
  // SerialBluetooth.println(state.getNextAction());

  if(state.RobotMode() == TELEOP_MODE){ // if in teleop
    // handle drivetrain
    runDrivetrain();
    // handle state machine decisions    
    runStateSelector();

    // enable / disable field oriented driving
    if(PestoLink.buttonHeld(buttonEnableFieldOriented)){ 
      drivetrain.setDriveMode(FIELD_ORIENTED);
    }
    else if(PestoLink.buttonHeld(buttonDisableFieldOriented)){
      drivetrain.setDriveMode(ROBOT_ORIENTED);
    }
  }
  if(PestoLink.buttonHeld(buttonZeroYaw)){ // reset IMU yaw
    pose.zeroYaw();
  }
  // enable / disable logic
  if(PestoLink.buttonHeld(buttonEnable)){
    state.setEnable(ENABLE);
  }
  else if(PestoLink.buttonHeld(buttonDisable)){
    state.setEnable(DISABLE);
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
  // pathFollower.update();
  // shooterAim.update();

  // update from driver station
  if(!PestoLink.update()){
    state.setEnable(DISABLE); // disable if we disconnect
  }

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
  float linearX = deadzone(PestoLink.getAxis(axisLinX));
  float linearY = -deadzone(PestoLink.getAxis(axisLinY));
  float angularZ = deadzone(PestoLink.getAxis(axisAngZ)) * 0.5;
  
  drivetrain.drive(linearX, linearY, angularZ);

  return;
}

void runStateSelector(){
  // this giant mess of if statements handles all the state transitions
  if(PestoLink.buttonHeld(buttonIntake) && state.getNextAction() <= INTAKE && !state.hasNote()){ // we want to only go to an intake state if we're not currently doing something else
    state.setNextAction(INTAKE);
    justIntake = true;
  }
  else if(PestoLink.buttonHeld(buttonSource)){ // manual "reverse"
    state.setNextAction(SOURCE);
    justIntake = true;
  }
  else if(justIntake && 
          (!PestoLink.buttonHeld(buttonIntake) && state.getNextAction() <= INTAKE) || 
          (!PestoLink.buttonHeld(buttonSource) && state.getNextAction() == SOURCE)){ // intake should stop if we let go of the button for source or ground
    state.setNextAction(STOP);
    justIntake = false;
  }
  else if(PestoLink.buttonHeld(buttonSub)){
    state.setNextAction(SUBWOOFER);
  }
  else if(PestoLink.buttonHeld(buttonAmpForward)){
    state.setNextAction(AMP_FORWARD);
  }
  else if(PestoLink.buttonHeld(buttonPass)){
    state.setNextAction(PASS);
  }
  else if(PestoLink.buttonHeld(buttonAmpBackward)){
    state.setNextAction(AMP_BACKWARD);
  }
  else if(PestoLink.buttonHeld(buttonDynamic)){
    state.setNextAction(DYNAMIC);
  }
  else if(PestoLink.buttonHeld(buttonClimbUp)){
    state.setNextAction(CLIMBERS_UP);
  }
  else if(PestoLink.buttonHeld(buttonClimbDown)){
    state.setNextAction(CLIMBERS_DOWN);
  }

  // we only want to do this if we're currently in another state
  // (as in, if we are trying to shoot)
  if(PestoLink.buttonHeld(buttonIntake) && state.getNextAction() > INTAKE){ 
    intake.execute();
  }

  // actually have the execute button do it's thing
  if(PestoLink.buttonHeld(buttonExecute)){
    arm.execute();
    shooter.execute();
    justExecuted = true;
  }
  if(justExecuted && !PestoLink.buttonHeld(buttonExecute) && state.getNextAction() > INTAKE){
    state.setNextAction(STOP);
    justExecuted = false;
  }
}

