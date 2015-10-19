#include "SampleOdometryModel.hpp"
#include <math.h>

// basic constructor
SampleOdometryModel::SampleOdometryModel(
                                            const ros::NodeHandle &private_nh,
                                            CommandOdom *cmd_input
                                        ) : SampleMotionModel(), cmds(cmd_input), old_odom(), odom()  {

    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param("sample_odometry_model_alpha_1", alpha1, 0.005);
    private_nh.param("sample_odometry_model_alpha_2", alpha2, 0.001);
    private_nh.param("sample_odometry_model_alpha_3", alpha3, 0.001);
    private_nh.param("sample_odometry_model_alpha_4", alpha4, 0.005);
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
    double rot1, trans, rot2, rot1_hat, trans_hat, rot2_hat, x_, y_, theta_, *sample_pose;

    //pose para atualizar
    sample_pose = p->v;

    //calc rot1
    rot1  = angleDiff(atan2(odom.v[1] - old_odom.v[1], odom.v[0] - old_odom.v[0]) , old_odom.v[2]);

    //calculate trans and test if moved
    trans = sqrt(pow(odom.v[0] - old_odom.v[0],2) + pow(odom.v[1] - old_odom.v[1],2));

    if(trans > 0.001){
        //calc rot2
        rot2 = angleDiff(angleDiff(odom.v[2], old_odom.v[2]), rot1);

        //Noise
        //trocar
        rot1_hat = angleDiff(gaussianPDF(alpha1 * fabs(rot1) + alpha2 * fabs(trans))*0.5,rot1);

        //anglediff
        trans_hat = trans - gaussianPDF(alpha3*fabs(trans)+ alpha4 * fabs(angleDiff(rot1, rot2)))*0.5;

        //trocar
        rot2_hat = angleDiff( gaussianPDF(alpha1*fabs(rot2) + alpha2 * fabs(trans))*0.5,rot2);


        //update the command odom
        sample_pose[0] +=  trans_hat * cos(sample_pose[2]+rot1_hat);
        sample_pose[1] +=  trans_hat * sin(sample_pose[2]-rot1_hat);// - rot1
        sample_pose[2] = normalize(sample_pose[2]+ angleDiff(rot1_hat, rot2_hat));
    //    ROS_INFO("sample X: [%f]       [%f]\n", sample_pose[0]);
    //    ROS_INFO("sample Y: [%f]       [%f]\n", sample_pose[1]);
    //    ROS_INFO("sample Theta: [%f]       [%f]\n\n", sample_pose[2]);

        if (trans_hat > 0.0 || rot2_hat != 0.0) {
            moved = true;
        }
        
    } 

}

// updates the commands
void SampleOdometryModel::update(const ros::Time &end) {

    //get the commands (ut<xt-1, xt>)
    std::vector<Pose2D> commands = cmds->getCommandOdom(end);

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
