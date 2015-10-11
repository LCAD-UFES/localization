#ifndef SAMPLE_MOTION_MODEL_VELOCITY_H
#define SAMPLE_MOTION_MODEL_VELOCITY_H

#include <vector>

#include "CommandVelocity.hpp"
#include "SampleMotionModel.hpp"

class SampleVelocityModel : public SampleMotionModel {

    // Sample Velocity Model parameters 
    double a1, a2, a3, a4, a5, a6;
    double deltaT;

    // the command
    CommandVel *cmds;

    public:
        // default constructor
        SampleVelocityModel(ros::NodeHandle&, CommandVel *);
        // destructor
        ~SampleVelocityModel();
        // updates the pose to a new one based on the command
        virtual void samplePose2D(SampleSet*);
};

#endif