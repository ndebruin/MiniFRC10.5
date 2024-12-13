#include "PoseEstimator.h"

#ifndef FIELDDIMS_h
#define FIELDDIMS_h

#define fieldLength 144.0 // inches
#define fieldWidth 70.880 // inches

// x+ is towards source from amp wall



// blue scoring targets
constexpr Pose SpeakerBlue = 
{
    24.0, // x
    1.57, // y 
    180.0 // theta
};

constexpr Pose AmpBlue = 
{
    0.25, // x
    17.75, // y 
    270.0 // theta
};

constexpr Pose PassBlue = 
{
    8.0, // x
    15.0, // y 
    180.0 // theta
};

constexpr Pose SourceBlue = {
    fieldWidth, // fake x
    fieldLength, // fake y
    60.0 // theta
};

// red scoring targets
constexpr Pose SpeakerRed = 
{
    24.0, // x
    fieldLength-1.57, // y 
    0.0 // theta
};

constexpr Pose AmpRed = 
{
    0.25, // x
    fieldLength-17.75, // y 
    270.0 // theta
};

constexpr Pose PassRed = 
{
    8.0, // x
    fieldLength-15.0, // y 
    0.0 // theta
};

constexpr Pose SourceRed = {
    fieldWidth, // fake x
    0.0, // fake y
    120.0 // theta
};


#endif //FIELDDIMS_h