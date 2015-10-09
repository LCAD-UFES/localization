#ifndef POSE_3D_H
#define POSE_3D_H

// Pose3D, [0] == x, [1] == y and [2] == z, [3]== rool, [4] == pitch and [5] == yaw
struct Pose3D {
    double v[6] = {0, 0, 0, 0, 0, 0};
};

#endif