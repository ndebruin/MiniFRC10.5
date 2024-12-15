#include "PoseEstimator.h"

#ifndef AUTONDEFS_h
#define AUTONDEFS_h

// subwoofer starting positions
constexpr Pose sourceSideSubBlue = 
{
    34.33, // x
    5.85, // y 
    240.0 // theta
};

constexpr Pose centerSideSubBlue = 
{
    24.0, // x
    11.7, // y 
    180.0 // theta
};

constexpr Pose ampSideSubBlue = 
{
    14.0, // x
    5.85, // y 
    120.0 // theta
};

// spike note locations
constexpr Pose sourceSpikeNoteBlue = 
{
    36.0, // x
    27.25, // y 
    180.0 // theta
};

constexpr Pose centerSpikeNoteBlue = 
{
    24.0, // x
    27.25, // y 
    180.0 // theta
};

constexpr Pose ampSpikeNoteBlue = 
{
    12.0, // x
    27.25, // y 
    180.0 // theta
};






// subwoofer starting positions
constexpr Pose sourceSideSubRed = 
{
    34.33, // x
    fieldLength - 5.85, // y 
    300.0 // theta
};

constexpr Pose centerSideSubRed = 
{
    24.0, // x
    fieldLength - 11.7, // y 
    0.0 // theta
};

constexpr Pose ampSideSubRed = 
{
    14.0, // x
    fieldLength - 5.85, // y 
    180.0 // theta
};

// spike note locations
constexpr Pose sourceSpikeNoteRed = 
{
    36.0, // x
    fieldLength - 27.25, // y 
    0.0 // theta
};

constexpr Pose centerSpikeNoteRed = 
{
    24.0, // x
    fieldLength - 27.25, // y 
    0.0 // theta
};

constexpr Pose ampSpikeNoteRed = 
{
    12.0, // x
    fieldLength - 27.25, // y 
    0.0 // theta
};


// centerline notes



#endif //AUTONDEFS_H
