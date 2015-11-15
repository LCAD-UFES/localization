#ifndef SAMPLE_MOTION_MODEL_VELOCITY_H
#define SAMPLE_MOTION_MODEL_VELOCITY_H

#include <vector>

#include "CommandVelocity.hpp"
#include "SampleMotionModel.hpp"
#include "Velocity.hpp"

#include "Pose2D.hpp"

#include "wrap2pi.h"

class SampleVelocityModel : public SampleMotionModel {

    // Sample Velocity Model parameters 
    double a1, a2, a3, a4, a5, a6;

    // the CommandVel reader
    CommandVel *cmds;

    // the command
    std::vector<Velocity> commands;

    // a pi value
    double PI2;

    public:
        // default constructor
        SampleVelocityModel(ros::NodeHandle&, CommandVel *);

        // destructor
        ~SampleVelocityModel();

        // updates the pose to a new one based on the command
        virtual void samplePose2D(Pose2D*);

        // update the commands
        virtual bool update(const ros::Time&);

};

#endif