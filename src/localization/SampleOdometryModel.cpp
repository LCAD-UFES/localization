#include "SampleOdometryModel.hpp"
#include <math.h>

// basic constructor
SampleOdometryModel::SampleOdometryModel(
                                            const ros::NodeHandle &private_nh,
                                            CommandOdom *cmd_input
                                        ) : SampleMotionModel(), cmds(cmd_input), commands(), old_odom(), odom()  {

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
/*
 * Sample Odometry Model Algoritm, sample Odometry poses
 * Table 5.4
*/
void SampleOdometryModel::samplePose2D(Pose2D *p) {
    // auxiliar variables
    double rot1, trans, rot2, rot1_hat, trans_hat, rot2_hat, x_, y_, theta_, *samplePose;

    //pose para atualizar
    samplePose = p->v;
    //calc rot1
    rot1  = angleDiff(atan2(odom.v[1] - old_odom.v[1], odom.v[0] - old_odom.v[0]) , old_odom.v[2]);
    //calc trans and test if move
    trans = sqrt(pow(odom.v[0] - old_odom.v[0],2) + pow(odom.v[1] - old_odom.v[1],2));
      if(trans<0.0001){
          trans = 0.0;
      }
      rot2 = angleDiff(angleDiff(odom.v[2], old_odom.v[2]), rot1);

    //update the command odom



}

// updates the commands
void SampleOdometryModel::update(const ros::Time&) {
    if (!commands.empty()) {
        commands.clear();
    }
    //get the commands (ut<xt-1, xt>)
    commands = cmds->getCommandOdom;
    //more easy to read
    old_odom = commands[0];
    odom = commands[1];
}
//Reduce call to function anglDistance, return diference between two angles
double SampleOdometryModel::angleDiff(double a, double b){
   return mrpt::math::angDistance(a,b);
}
//Reduce call to function wrapToPi, return the angle [-pi, pi]
double SampleOdometryModel::normalize(double a){
    return mrpt::math::wrapToPi(a);
}
