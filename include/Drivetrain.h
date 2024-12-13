#ifndef Drivetrain_h
#define Drivetrain_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "PoseEstimator.h"


#include "State.h"

#define FIELD_ORIENTED true
#define ROBOT_ORIENTED false

class Drivetrain
{
    public:
        Drivetrain(NoU_Drivetrain* NoUDrivetrain, PoseEstimator* poseEstimator, State* robotState);
        uint8_t begin();
        uint8_t update();

        bool getFieldOriented(){ return fieldOriented; }

        void setFieldOriented(bool FieldOriented){ fieldOriented = FieldOriented; }

        // will either be field oriented or not depending on the seperately set field (setFieldOriented(bool))
        void drive(float linearX, float linearY, float angularZ);

        // will either be field oriented or not depending on the passed boolean
        void drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled); 

        void stop();


        void setX(float X){ desiredX = X; }
        void setY(float Y){ desiredY = Y; }
        void setTheta(float theta){ desiredTheta = theta; }

    private:
        NoU_Drivetrain* nouDrivetrain;

        PoseEstimator* pose;
        State* robotState;

        bool fieldOriented = false;

        float desiredX;
        float desiredY;
        float desiredTheta;
 
};

#endif