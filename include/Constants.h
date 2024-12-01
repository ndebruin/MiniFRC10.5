#define robotName "Team 43"

////////////////////////////////////////////////////////////////////// Drivetrain //////////////////////////////////////////////////////////////////////

#define frontLeftMotorChannel 1
#define frontRightMotorChannel 2
#define backLeftMotorChannel 3
#define backRightMotorChannel 4

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

#define shooterMotorChannel 5

#define Shooter_AMP_Forward_kS 0.0
#define Shooter_AMP_Backward_kS 0.0

#define Shooter_SUBWOOFER_kS 0.0

#define Shooter_DYNAMIC_kS 0.0

#define Shooter_PASS_kS 0.0

#define Shooter_SOURCE_kS -0.0

////////////////////////////////////////////////////////////////////// Indexer //////////////////////////////////////////////////////////////////////

#define intakeMotorChannel 6

#define Intake_IN_kS 0.0
#define Intake_SHOOT_kS 0.0

#define Intake_OUT_kS -0.0

#define pinSensor 4
#define pinFeedbackLED 5

#define sensorValueEmpty 100
#define sensorValueNote 1000

////////////////////////////////////////////////////////////////////// Arm //////////////////////////////////////////////////////////////////////

#define armServoChannel 1

#define arm_STOW_angle 0

#define arm_SUB_angle 0

#define arm_AMP_FORWARD_angle 0
#define arm_AMP_BACKWARD_angle 0

#define arm_PASS_angle 0

#define arm_CLIMB_DEPLOY_angle 0
#define arm_CLIMB_RETRACT_angle 0

////////////////////////////////////////////////////////////////////// Controller //////////////////////////////////////////////////////////////////////

#define deadzoneValue 0.05

#define axisLinY 1
#define axisLinX 0
#define axisAngZ 2

#define buttonIntake 6 
#define buttonExecute 7

#define buttonAmp 1
#define buttonPass 2
#define buttonSub 0
#define buttonPodium 3
#define buttonSource 4

#define buttonClimb 5
#define buttonDeployReaction 15
#define buttonStowReaction 14

#define buttonZeroYaw 9