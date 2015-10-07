#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : SampleMotionModel(), last_cmd(), cmd_queue(), spinner(1, &cmd_queue) {
    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param<float>("sample_velocity_model_alpha_1", a1, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_2", a2, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_3", a3, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_4", a4, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_5", a5, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_6", a6, 0.1);

    private_nh.setParam<double>("ns", valor);
    // the deltaT
    private_nh.param<float>("sample_velocity_model_delta_t", deltaT, 0.1);

    // get the topic name
    std::string topic;
    private_nh.param<std::string>("sample_velocity_model_cmd_topic", topic, "cmd_vel");
    // get the queue size
    float queue_size;
    private_nh.param<float>("sample_velocity_model_queue_size", queue_size, 10);

    // Subscribe the topic with the appropriated async spinner
    // configure the subscribe options
    ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(topic, queue_size, &SampleVelocityModel::listenCMD, this, cmd_queue);
    // Subscribe
    sub = nh.subscribe(ops);
}
// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(Pose2D *pose) {

    // auxiliar variables
    double v, w, y, vw;

    // get the last cmd message
    readCMD();

    // if angular.z is zero, we have an undefined case, so...
    if (0.0 != last_cmd.angular.z) {

        v = last_cmd.linear.x + gaussianPDF(a1*last_cmd.linear.x*last_cmd.linear.x + a2*last_cmd.angular.z*last_cmd.angular.z);
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
        pose->v[2] = pose->v[2] + w*deltaT + y*deltaT;
    }


}

// the callback method to listen the command 
// in this case it gets a Twist message and assigns to last_msg private attribute
// we need to implement a real world solution
// for now, it's just to start and learn how to use Object methods as a Subscriber callback
void SampleVelocityModel::listenCMD(const geometry_msgs::TwistStamped msg) {
    last_cmd = msg;
}

// necessary to process our callbacks
void SampleVelocityModel::readCMD() {
    cmd_queue.callAvailable(0);
}