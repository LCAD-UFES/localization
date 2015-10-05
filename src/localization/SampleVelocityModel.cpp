#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel() : SampleMotionModel(), cmd("cmd_vel", 1000) {
    a1 = a2 = a3 = a4 = a5 = a6 = ALPHA_1;
    deltaT = 0.1;
}

// constructor
SampleVelocityModel::SampleVelocityModel(std::string topic_name, unsigned int q_size) : SampleMotionModel(), cmd(topic_name, q_size) {
    a1 = a2 = a3 = a4 = a5 = a6 = ALPHA_1;
    deltaT = 0.1;
}

// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(Pose2D *pose) {

    // auxiliar variables
    double v, w, y, vw;

    v = cmd->v + sampler.sample(a1*cmd->v*cmd->v + a2*cmd->w*cmd->w);
    w = cmd->w + sampler.sample(a3*cmd->v*cmd->v + a4*cmd->w*cmd->w);
    y = sample.sample(a5*cmd->v*cmd->v + a6*cmd->w*cmd->w);

    vw = v/w;

    pose->v[0] = pose->v[0] - vw*sin(pose->v[2]) + vw*sin(pose->v[2] + w*deltaT);
    pose->v[1] = pose->v[1] + vw*cos(pose->v[2]) - vw*cos(pose->v[2] + w*deltaT);
    pose->v[2] = pose->v[2] + w*deltaT + y*deltaT;
}