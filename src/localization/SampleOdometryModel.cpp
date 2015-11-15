#include "SampleOdometryModel.hpp"
#include <math.h>

// basic constructor
SampleOdometryModel::SampleOdometryModel(
        const ros::NodeHandle &private_nh,
        CommandOdom *cmd_input
        ) : SampleMotionModel(), cmds(cmd_input), old_odom(), odom()  {

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param("sample_odometry_model_alpha_1", alpha1, 0.001);
    private_nh.param("sample_odometry_model_alpha_2", alpha2, 0.001);
    private_nh.param("sample_odometry_model_alpha_3", alpha3, 0.001);
    private_nh.param("sample_odometry_model_alpha_4", alpha4, 0.001);
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
    double rot1, trans, rot2, rot1_hat, trans_hat, rot2_hat, *sample_pose;

    //pose para atualizar
    sample_pose = p->v;
    if(moved){
        //calculate trans and test if moved
        trans = sqrt(pow(odom.v[0] - old_odom.v[0],2) + pow(odom.v[1] - old_odom.v[1],2));

        //calc rot1
        rot1 = mrpt::math::wrapToPi(atan2(odom.v[1] - old_odom.v[1], odom.v[0] - old_odom.v[0]) - old_odom.v[2]);


        if(trans < 0.0001){
            trans = 0.0;
        }

        //calc rot2
        rot2 = mrpt::math::angDistance(mrpt::math::angDistance(odom.v[2], old_odom.v[2]), rot1);

        //Noise
        // change
        rot1_hat = mrpt::math::wrapToPi(rot1 + gaussianPDF(alpha1 * fabs(rot1) + alpha2 * fabs(trans)));

        //anglediff
        trans_hat = trans + gaussianPDF(alpha3*fabs(trans)+ alpha4 * fabs(mrpt::math::angDistance(rot1, rot2)));

        // change
        rot2_hat = mrpt::math::wrapToPi(rot2+ gaussianPDF(alpha1*fabs(rot2) + alpha2 * fabs(trans)) );



        //update the command odom
        sample_pose[0] +=  trans_hat * cos(mrpt::math::wrapToPi(sample_pose[2]+rot1_hat));
        sample_pose[1] +=  trans_hat * sin(mrpt::math::wrapToPi(sample_pose[2]+rot1_hat));
        sample_pose[2] = mrpt::math::wrapToPi(sample_pose[2]+ mrpt::math::angDistance(rot1_hat, rot2_hat));

    }

}

// updates the commands
bool SampleOdometryModel::update(const ros::Time &end) {

    //get the commands (ut<xt-1, xt>)
    std::vector<Pose2D> commands = cmds->getCommandOdom(end, moved);
    //more easy to read
    old_odom = commands[0];
    odom = commands[1];

    return moved;

}

//Reduce call to function anglDistance, return diference between two angles
double SampleOdometryModel::angleDiff(double a, double b){
    return mrpt::math::angDistance(a,b);
}

// Reduce call to function wrapToPi, return the angle [-pi, pi]
// you don't need this function
double SampleOdometryModel::normalize(double a){
    return mrpt::math::wrapToPi(a);
}
