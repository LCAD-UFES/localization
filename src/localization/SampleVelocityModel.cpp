#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel(
                                            ros::NodeHandle &private_nh,
                                            std::vector<CommandReader *> cmd_input
                                        ) : SampleMotionModel(),  cmds(cmd_input) {

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param<float>("sample_velocity_model_alpha_1", a1, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_2", a2, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_3", a3, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_4", a4, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_5", a5, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_6", a6, 0.1);

}

SampleVelocityModel::~SampleVelocityModel() {
    cmds = nullptr;
}
// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(Pose2D *pose) {

    // auxiliar variables
    double v, w, y, vw;

    // get the last cmd message
    readCMD();

    geometry_msgs::TwistStamped cmd_vel =

    // if angular.z is zero, we have an undefined case, so...
    if (0.0 != last_cmd.angular.z) {

        v = cmd.linear.x + gaussianPDF(a1*last_cmd.linear.x*last_cmd.linear.x + a2*last_cmd.angular.z*last_cmd.angular.z);
        w = last_cmd.angular.z. + gaussianPDF(a3*last_cmd.linear.x*last_cmd.linear.x + a4*last_cmd.angular.z*last_cmd.angular.z);
        y = gaussianPDF(a5*last_cmd.linear.x*last_cmd.linear.x + a6*last_cmd.angular.z*last_cmd.angular.z);

        vw = v/w;

        pose->v[0] = pose->v[0] - vw*sin(pose->v[2]) + vw*sin(pose->v[2] + w*deltaT);
        pose->v[1] = pose->v[1] + vw*cos(pose->v[2]) - vw*cos(pose->v[2] + w*deltaT);
        pose->v[2] = pose->v[2] + w*deltaT + y*deltaT;

    }else {
        v = last_cmd.linear.x + gaussianPDF(a1*last_cmd.linear.x*last_cmd.linear.x);
        w = gaussianPDF(a3*last_cmd.linear.x*last_cmd.linear.x);
        y = gaussianPDF(a5*last_cmd.linear.x*last_cmd.linear.x);

        pose->v[0] = pose->v[0] + v*deltaT*cos(pose[2]);
        pose->v[1] = pose->v[1] + v*deltaT*sin(pose[2]);
        pose->v[2] = pose->v[2] + y*deltaT;
    }
}