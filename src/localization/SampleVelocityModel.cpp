#include "SampleVelocityModel.hpp"
#include <cmath>

// Default constructor
SampleVelocityModel::SampleVelocityModel(
                                            ros::NodeHandle &private_nh,
                                            CommandVel *cmd_input
                                        ) : SampleMotionModel(),  cmds(cmd_input), commands() {

    // the pi value
    PI2 = M_PI*2.0;

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param("sample_velocity_model_alpha_1", a1, 0.1);
    private_nh.param("sample_velocity_model_alpha_2", a2, 0.1);
    private_nh.param("sample_velocity_model_alpha_3", a3, 0.05);
    private_nh.param("sample_velocity_model_alpha_4", a4, 0.05);
    private_nh.param("sample_velocity_model_alpha_5", a5, 0.05);
    private_nh.param("sample_velocity_model_alpha_6", a6, 0.05);

}

// destructor
// avoiding to delete the cmds pointer

SampleVelocityModel::~SampleVelocityModel() {
    cmds = nullptr;
}
// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(Pose2D *p) {

    // auxiliar variables
    double v, w, y, v2, w2, dt, vw, *pose;

    // get the current pose
    pose = p->v;

    // iterate over the Velocity Commands
    for (int j = 0; j < commands.size() - 1; j++) {

        // just to be easy to write and reduce repetitive multiplication
        v2 = std::fabs(commands[j].linear);
        w2 = std::fabs(commands[j].angular);

        // the time betwen the current and the next command
        // the last command in this command vector is a hack
        // just to obtain the time from the laser and set the last command as
        // the first one in the next MCL call (next LaserScan)
        dt = (commands[j+1].stamp - commands[j].stamp).toSec();

        // get the linear velocity
        v = commands[j].linear*1.2 + gaussianPDF(a1*v2 + a2*w2);

        // get the angular velocity
        w = commands[j].angular*1.50 + gaussianPDF(a3*v2 + a4*w2);

        // get the final angle extra noisy angular velocity
        y = gaussianPDF(a5*v2 + a6*w2);

        // updates the pose based on this current command
        // verify if the angular is zero or very close to zero

        if (0.0 != commands[j].angular) {

            // here we can use the given algorithm directly
            vw = v/w;
            // get the x distance
            pose[0] += - vw*sin(pose[2]) + vw*sin(pose[2] + w*dt);
            // get the y distance
            pose[1] += (vw*cos(pose[2]) - vw*cos(pose[2] + w*dt));
            // get the new angle
            pose[2] += w*dt + y*dt;


            // indicates the movement
            moved = true;

        } else if (0.0 != commands[j].linear) {

            // get the x distance
            pose[0] += v*cos(pose[2])*dt;
            // get the y distance
            pose[1] += v*sin(pose[2])*dt;
            // get the new angle, just adding some noise
            pose[2] += y*dt;

            // indicates the movement
            moved = true;

        } else {

            // indicates the movement
            moved = false;

        }
        // angular velocity
        // maintain the orientation between -PI and PI
        if (-M_PI > pose[2]) {
            pose[2] += PI2;
        } else if (M_PI < pose[2]) {
            pose[2] -= PI2;
        }

    }

}

// update the commands
bool SampleVelocityModel::update(const ros::Time &end) {

    if (!commands.empty()) {
        commands.clear();
    }
    //get the correct commands
    commands = cmds->getAll(end);

    if (commands.size() == 2 && 0.0 == commands[0].linear && 0.0 == commands[0].angular) {

        // verify if the commands are different
        return commands[0].linear != commands[1].linear && commands[0].angular != commands[1].angular;

    } else {

        // in this case, return true, the commands.size() is greater than 2 or the commands are diferents
        return true;

    }

}