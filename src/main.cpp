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
#include "AutonRunner.h"
#include "DynamicShotController.h"

#include "Constants.h"

////////////////////////////////////////////////////////////////////// Hardware Declarations //////////////////////////////////////////////////////////////////////

// BluetoothSerial SerialBluetooth; // bluetooth link

State state;

PoseEstimator pose(&Serial, baudRate, &state); // we're just running the coproc serial over the normal serial bus.
                                       // this kinda sucks bc it means that we need to disconnect it whenever we program, but oh well
                                       // can still use Serial.println() for normal computer info tho

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

DynamicShotController shooterAim = DynamicShotController(&drivetrain, &arm, &shooter, &pose, &state);

AutonRunner autonRunner = AutonRunner(&drivetrain, &arm, &shooter, &intake, &pose, &state);

////////////////////////////////////////////////////////////////////// Function Declerations //////////////////////////////////////////////////////////////////////

void asyncUpdate();

double deadzone(double raw);
void constantButtons();
void runDrivetrain();
void runStateSelector(); // shot presets, climber, and intake

void updatePestoLink();

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
  shooterAim.begin();
  autonRunner.begin(asyncUpdate);
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

void loop() 
{
  asyncUpdate(); // updates all the things that need to be updated every loop regardless of anything else
  constantButtons(); // handle all the button inputs that should always happen

  // Serial.println("sensor1:" + String(intake.sensor1Value()) + " sensor2:" + String(intake.sensor2Value()));

  // Serial.println(String(pose.getCurrentGlobalPose().x) + "x" + String(pose.getCurrentGlobalPose().y) + "y" + String(pose.getCurrentGlobalPose().yaw) + "t");
  // SerialBluetooth.println(state.getNextAction());



  if(!state.isEnabled()){ // reuse preset buttons for auton selectors
    if(PestoLink.buttonHeld(buttonSub)){
      state.selectedAuton = 1;
    }
    else if(PestoLink.buttonHeld(buttonAmpForward)){
      state.selectedAuton = 2;
    }
    else if(PestoLink.buttonHeld(buttonPass)){
      state.selectedAuton = 3;
    }
    else if(PestoLink.buttonHeld(buttonAmpBackward)){
      state.selectedAuton = 4;
    }
  }

  if(state.getRobotMode() == TELEOP_MODE && state.isEnabled()){ // if in teleop
    // handle drivetrain
    runDrivetrain();
    // handle state machine decisions    
    runStateSelector();
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
  autonRunner.update();
  shooterAim.update();

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

  // update pestolink telem
  updatePestoLink();
}

void updatePestoLink(){
  String telemString;

  if(state.getAlliance() == RED){
    telemString.concat('R');
  }
  else{
    telemString.concat('B');
  } // 2/8 characters
  telemString.concat(' ');
  telemString.concat(state.getNextAction());
  telemString.concat(' '); // 4/8 characters
  telemString.concat(String(pose.getYaw()));

  if(state.isDynamic() == DYNAMIC){
    telemString.concat('D');
  }
  else{
    telemString.concat('P');
  } // 5/8 characters

  char* telem = (char*)telemString.c_str();
  // 8/8 characters

   // update pestolink telem
  if(state.getAlliance() == BLUE){
    PestoLink.print(telem, "0000FF");
  }
  else{
    PestoLink.print(telem, "FF0000");
  }
  return;
}

double deadzone(double rawJoy){
  if(fabs(rawJoy) < deadzoneValue){
    return 0.0;
  }
  return rawJoy;
}

void constantButtons(){
  // enable / disable logic
  // another "oh fuck i ran out of buttons" moment
  if(PestoLink.buttonHeld(buttonZeroYaw) && PestoLink.buttonHeld(buttonEnable)){
    state.setEnable(DISABLE);
  }
  else if(PestoLink.buttonHeld(buttonZeroYaw)){ // reset IMU yaw
    pose.zeroYaw();
  }
  else if(PestoLink.buttonHeld(buttonEnable)){
    state.setEnable(ENABLE);
  }

  if(!state.isEnabled()){ // holy shit i'm literally running out of buttons i hate this so much
    if(PestoLink.getAxis(axisAlliance) > 0.9){
      state.setAlliance(RED);
    }
    if(PestoLink.getAxis(axisAlliance) < -0.9){
      state.setAlliance(BLUE);
    }
  }
  // holy crap this is a really bad overload
  if(PestoLink.buttonHeld(buttonClimbDown)){
    state.setTeleopMode();
  }

  // enable / disable field oriented driving
  if(PestoLink.buttonHeld(buttonEnableFieldOriented)){ 
    drivetrain.setFieldOriented(FIELD_ORIENTED);
  }
  else if(PestoLink.buttonHeld(buttonDisableFieldOriented)){
    drivetrain.setFieldOriented(ROBOT_ORIENTED);
  }
}

void runDrivetrain(){
  float linearX = deadzone(PestoLink.getAxis(axisLinX));
  float linearY = -deadzone(PestoLink.getAxis(axisLinY));
  float angularZ = deadzone(PestoLink.getAxis(axisAngZ)) * 0.5;
  
  drivetrain.teleopDrive(linearX, linearY, angularZ);

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
  else if(PestoLink.buttonHeld(buttonClimbUp)){
    state.setNextAction(CLIMBERS_UP);
  }
  else if(PestoLink.buttonHeld(buttonClimbDown)){
    state.setNextAction(CLIMBERS_DOWN);
  }

  // if(PestoLink.buttonHeld(buttonDynamicEnable)){
  //   state.setDynamicTargeting(DYNAMIC);
  // }
  else if(PestoLink.buttonHeld(buttonDynamicDisable)){
    state.setDynamicTargeting(PRESET);
  }

  // we only want to do this if we're currently in another state
  // (as in, if we are trying to shoot)
  if(PestoLink.buttonHeld(buttonIntake) && (state.getNextAction() > INTAKE && state.getNextAction() != SOURCE)){ 
    intake.execute();
  }

  // actually have the execute button do it's thing
  if(PestoLink.buttonHeld(buttonExecute) && (state.isDynamic() == PRESET)){
    arm.execute();
    shooter.execute();
    justExecuted = true;
  }
  else if(PestoLink.buttonHeld(buttonExecute) && (state.isDynamic() == DYNAMIC)){
    shooterAim.execute();
  }
  if(justExecuted && !PestoLink.buttonHeld(buttonExecute) && state.getNextAction() > INTAKE){
    state.setNextAction(STOP);
    justExecuted = false;
  }
}

