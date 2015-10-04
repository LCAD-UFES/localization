#ifndef POSE_H
#define POSE_H

#include "VectorT.hpp"

// VectorT_DBL_3 and VectorT_DBL_6 are defined in vectorT.hpp
// double Pose2D[3]
// x, y and roll
typedef VectorT_DBL_3 Pose2D;

// double Pose3D[6]
// x, y, roll, pitch and yaw
typedef VectorT_DBL_6 Pose3D;

#endif