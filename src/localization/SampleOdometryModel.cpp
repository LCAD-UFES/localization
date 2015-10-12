#include "SampleOdometryModel.hpp"

// basic constructor
SampleOdometryModel::SampleOdometryModel(
                                            const ros::NodeHandle &private_nh,
                                            CommandOdom *cmd_input
                                        ) : SampleMotionModel(), cmds(cmd_input) {

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param("sample_odometry_model_alpha_1", a1, 0.1);
    private_nh.param("sample_odometry_model_alpha_2", a2, 0.1);
    private_nh.param("sample_odometry_model_alpha_3", a3, 0.1);
    private_nh.param("sample_odometry_model_alpha_4", a4, 0.1);
}

// basic destructor 
SampleOdometryModel::~SampleOdometryModel() {
    // the CommandOdom pointer shoudl be managed inside the ParticleFilter class
    cmds = nullptr;
}

void SampleOdometryModel::samplePose2D(Pose2D*) {
    // auxiliar variables
    double rot1, trans, rot2, rot1_, trans_, rot2_, x_, y_, theta_;
    /* TODO */

}

// updates the commands
void SampleOdometryModel::updateCommands(const ros::Time&) {
    /* TODO */
}