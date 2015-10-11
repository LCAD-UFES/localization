#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel(
                                            ros::NodeHandle &private_nh,
                                            CommandVel *cmd_input
                                        ) : SampleMotionModel(),  cmds(cmd_input) {

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param("sample_velocity_model_alpha_1", a1, 0.1);
    private_nh.param("sample_velocity_model_alpha_2", a2, 0.1);
    private_nh.param("sample_velocity_model_alpha_3", a3, 0.1);
    private_nh.param("sample_velocity_model_alpha_4", a4, 0.1);
    private_nh.param("sample_velocity_model_alpha_5", a5, 0.1);
    private_nh.param("sample_velocity_model_alpha_6", a6, 0.1);

}

// destructor
// avoiding to delete the cmds pointer

SampleVelocityModel::~SampleVelocityModel() {
    cmds = nullptr;
}
// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(SampleSet *Xt) {

    // auxiliar variables
    double v, w, y, v2, w2, dt, vw, *pose;
    Sample2D *samples;

    // the cmds is a vector with, maybe, more than 1 Velocity command
    // the getAll() method remove all commmands arrived
    // before the last LaserScan
    std::vector<Velocity> commands(cmds->getAll());

    // get the samples pointer
    samples = Xt->samples;

    // iterate over all particles/SampleSet
    for(int i = 0; Xt->size; i++) {

        // get the current pose
        pose = samples->pose.v;

        // iterate over the Velocity Commands
        for (int j = 0; j < commands.size() - 1; j++) {

            // just to be easy to write and reduce repetitive multiplication
            v2 = commands[j].linear*commands[j].linear;
            w2 = commands[j].angular*commands[j].angular;

            // the time betwen the current and the next command
            // the last command in this command vector is a hack
            // just to obtain the time from the laser and set the last command as
            // the first one in the next MCL call (next LaserScan)
            dt = (commands[j+1].stamp - commands[j].stamp).toSec();

            // get the linear velocity
            v = commands[j].linear + gaussianPDF(a1*v2 + a2*w2);
            // get the angular velocity
            w = commands[j].angular + gaussianPDF(a3*v2 + a4*w2);
            // get the final angle extra noise
            y = gaussianPDF(a5*v2 + a6*w2);

            // updates the pose based on this current command
            // verify if the angular is zero
            if (0.0 != commands[j].angular) {

                // here we can use the given algorithm directly
                vw = commands[j].linear/commands[j].angular;
                // get the x distance
                pose[0] = pose[0] - vw*sin(pose[2]) + vw*sin(pose[2] + w*deltaT);
                // get the y distance
                pose[1] = pose[1] + vw*sin(pose[2]) - vw*sin(pose[2] + w*deltaT);
                // get the new angle
                pose[2] += commands[j].angular*dt + y*dt;

            } else {
                // get the x distance
                pose[0] += commands[j].linear*dt*cos(pose[2]);
                // get the y distance
                pose[1] += commands[j].linear*dt*sin(pose[2]);
                // get the new angle, just adding some noise
                pose[2] += y*dt;
            }
        }
    }
}