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

#define kV 0.7 // minimum value to overcome friction 
               // determined experimentally

#define driveExp 1.5 // control "squaring" value to get more control at the low end

// strafe parameters
#define linX_kS 0.0
#define linX_kP 0.0
#define linX_kI 0.0
#define linX_kD 0.0

#define x_AcceptableError 0.0

// forward back parameters
#define linY_kS 0.0
#define linY_kP 0.0
#define linY_kI 0.0
#define linY_kD 0.0

#define y_AcceptableError 0.0

// turn parameters
#define angZ_kS 0.0
#define angZ_kP 0.0
#define angZ_kI 0.0
#define angZ_kD 0.0

#define theta_AcceptableError 0.0

////////////////////////////////////////////////////////////////////// Shooter //////////////////////////////////////////////////////////////////////

#define shooterMotorChannel 4

#define Shooter_AMP_Forward_kS 0.0
#define Shooter_AMP_Backward_kS 0.7

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

#define sensor1ValueNote 1100

#define sensor2ValueNote 1200

#define debounceTime 1

////////////////////////////////////////////////////////////////////// Arm //////////////////////////////////////////////////////////////////////

#define armServoChannel 1

#define arm_STOW_angle 22

#define arm_SUB_angle 67

#define arm_AMP_FORWARD_angle 180
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

#define buttonIntake 6
#define buttonExecute 7

#define buttonSource 4

#define buttonAmpForward 2
#define buttonPass 1
#define buttonSub 0
#define buttonAmpBackward 3
#define buttonDynamic 5

#define buttonClimbUp 12
#define buttonClimbDown 13

#define buttonZeroYaw 8
#define buttonEnableFieldOriented 10
#define buttonDisableFieldOriented 11

#define buttonEnable 9
#define buttonDisable 15

////////////////////////////////////////////////////////////////////// Pose Estimator //////////////////////////////////////////////////////////////////////
constexpr float MOUSE_CONVERSION_FACTOR = (1.0/1000.0); // 1000 Dots per inch

#define baudRate 115200
#define TXPin 5 // doesn't matter we're not transmitting lmao
#define RXPin 25

////////////////////////////////////////////////////////////////////// Dynamic Shot Controller //////////////////////////////////////////////////////////////////////
#define goalHeight 17 // inches
#define armHeight 4.6 // inches

constexpr float deltaHeight = (goalHeight - armHeight);

#define armRadius 2.6125 // inches

// our coordinates are at 0,0 in the corner between the amp and speaker on the blue side

// towards source is x+
// towards red is y+

#define fieldLength 144.0 // inches

#define goalXRed 24.0
constexpr float goalYRed = (fieldLength - 1.57);  // offset from far end of the field

#define goalXBlue 24.0
#define goalYBlue 1.57

#endif //CONSTANT_h