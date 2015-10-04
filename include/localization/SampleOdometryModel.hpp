#ifndef SAMPLE_ODOMETRY_MODEL_H
#define SAMPLE_ODOMETRY_MODEL_H

#include "SampleMotionModel.hpp"
#include "CommandOdometry.hpp"

class SampleOdometryModel : public SampleMotionModel {
    // parameters
    double a1, a2, a3, a4;

    // the PDF object
    // a normal distribution
    GaussianPDF sampler;

    // the command reader
    CommandOdom cmd;

    public:
        // constructor
        SampleOdometryModel();
        ~SampleOdometryModel();

        // updates the pose to a new one based on the CommandVel
        void samplePose2D(Pose2D *pose);
};

#endif