#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel(
                                            ros::NodeHandle &private_nh,
                                            std::vector<CommandReader *> *cmd_input
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
void SampleVelocityModel::samplePose2D(Pose2D *pose) {

}