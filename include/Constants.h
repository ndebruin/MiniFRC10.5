#ifndef CONSTANT_h
#define CONSTANT_h

#define robotName "Team 43"

////////////////////////////////////////////////////////////////////// Drivetrain //////////////////////////////////////////////////////////////////////

#define frontLeftMotorChannel 5
#define frontRightMotorChannel 1
#define backLeftMotorChannel 6
#define backRightMotorChannel 2 

#define strafeCompensation 0.9  // this helps with the fact that the back wheels are further out and therefore have more torque on the robot
                                // determined experimentally

#define kS 0.7 // minimum value to overcome friction 
               // determined experimentally

#define driveExp 1.5 // control "squaring" value to get more control at the low end

#define correctCount 10

// strafe parameters
#define linX_kS 0.1 //0.01
#define linX_kP 0.05 //0.01
#define linX_kI 0.0
#define linX_kD 0.0

#define x_AcceptableError 0.2 // inches

// forward back parameters
#define linY_kS 0.1 //0.01
#define linY_kP 0.05 //0.01
#define linY_kI 0.0
#define linY_kD 0.0

#define y_AcceptableError 0.2 // inches

// turn parameters
#define angZ_kS 0.001
#define angZ_kP 0.000000000000000000001
#define angZ_kI 0.0
#define angZ_kD 0.0

#define theta_AcceptableError 10.0 // degrees

////////////////////////////////////////////////////////////////////// Shooter //////////////////////////////////////////////////////////////////////

#define shooterMotorChannel 4

#define Shooter_AMP_Forward_kS 0.0
#define Shooter_AMP_Backward_kS 0.65

#define Shooter_SUBWOOFER_kS 1.0

#define Shooter_DYNAMIC_kS 1.0

#define Shooter_PASS_kS 1.0

#define Shooter_SOURCE_kS -1.0

////////////////////////////////////////////////////////////////////// Indexer //////////////////////////////////////////////////////////////////////

#define intakeMotorChannel 3

#define Intake_IN_kS 1.0
#define Intake_REVERSE_IN_kS -1.0

#define Intake_SHOOT_kS 1.0
#define Intake_AMP_kS -0.9

#define pinFeedbackLED 26

#define pinSensor1 35
#define pinSensor2 34

#define sensor1ValueNote 950

#define sensor2ValueNote 300

#define debounceTime 5

////////////////////////////////////////////////////////////////////// Arm //////////////////////////////////////////////////////////////////////

#define armServoChannel 1

#define arm_STOW_angle 22

#define arm_SUB_angle 73

#define arm_AMP_FORWARD_angle 200
#define arm_AMP_BACKWARD_angle 210

#define arm_PASS_angle 25

#define arm_SOURCE_angle 90

#define arm_CLIMB_DEPLOY_angle 182
#define arm_CLIMB_RETRACT_angle 40

////////////////////////////////////////////////////////////////////// Controller //////////////////////////////////////////////////////////////////////

#define deadzoneValue 0.1

#define axisLinY 1
#define axisLinX 0
#define axisAngZ 2

#define axisAlliance 3

#define buttonIntake 6 // left trigger
#define buttonExecute 7 // right trigger

#define buttonSource 4 // left bumper

#define buttonAmpForward 2 // abxy buttons
#define buttonPass 1
#define buttonSub 0
#define buttonAmpBackward 3

#define buttonDynamicEnable 14 // left dpad
#define buttonDynamicDisable 5 // right bumper

#define buttonClimbUp 12 // up dpad
#define buttonClimbDown 13 // down dpad

#define buttonZeroYaw 8 // select
#define buttonEnableFieldOriented 10 // left joystick click
#define buttonDisableFieldOriented 11 // right joystick click

#define buttonEnable 9 // start
#define buttonDisable 15 // right dpad

////////////////////////////////////////////////////////////////////// Pose Estimator //////////////////////////////////////////////////////////////////////
#define DPI 600.0 // 250 Dots per inch NEED TO TUNE

constexpr float MOUSE_CONVERSION_FACTOR = (1.0/DPI); 

#define baudRate 115200

////////////////////////////////////////////////////////////////////// Dynamic Shot Controller //////////////////////////////////////////////////////////////////////
#define goalHeight 17 // inches
#define armHeight 4.6 // inches

constexpr float deltaHeight = (goalHeight - armHeight);

#define armRadius 2.6125 // inches

#endif //CONSTANT_h