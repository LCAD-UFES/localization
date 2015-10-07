#include "SampleVelocityModel.hpp"

// Default constructor
SampleVelocityModel::SampleVelocityModel(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : SampleMotionModel() {
    // get the sample velocity model parameters parameters
    // the alphas
    private_nh.param<float>("sample_velocity_model_alpha_1", a1, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_1", a2, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_1", a3, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_1", a4, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_1", a5, 0.1);
    private_nh.param<float>("sample_velocity_model_alpha_1", a6, 0.1);

    // the deltaT
    private_nh.param<float>("sample_velocity_model_delta_t", deltaT, 0.1);

    // get the topic name
    std::string topic;
    private_nh.param<std::string>("sample_velocity_model_cmd_topic", topic, "cmd_vel");
    // get the queue size
    float queue_size;
    private_nh.param<float>("sample_velocity_model_queue_size", queue_size, 10);

    // subscribe to the correct topic
    // the subscribe function below accepts the public method address and and a pointer
    // "this"
    // overloaded subscribe method provided by ROS team
    // see the listenCMD method below
    sub = nh.subscribe(topic, queue_size, &SampleVelocityModel::listenCMD, this);

}

// sample a new pose from a given command and previous pose
// see Table 5.3 - Probabilistic Robotics
void SampleVelocityModel::samplePose2D(Pose2D *pose) {

    // auxiliar variables
    double v, w, y, vw;

    v = cmd.linear.x + sampler.sample(a1*cmd.linear.x*cmd.linear.x + a2*cmd.angular.z*cmd.angular.z);
    w = cmd.angular.z. + sampler.sample(a3*cmd.linear.x*cmd.linear.x + a4*cmd.angular.z*cmd.angular.z);
    y = sampler.sample(a5*cmd.linear.x*cmd.linear.x + a6*cmd.angular.z*cmd.angular.z);

    vw = v/w;

    pose->v[0] = pose->v[0] - vw*sin(pose->v[2]) + vw*sin(pose->v[2] + w*deltaT);
    pose->v[1] = pose->v[1] + vw*cos(pose->v[2]) - vw*cos(pose->v[2] + w*deltaT);
    pose->v[2] = pose->v[2] + w*deltaT + y*deltaT;

}

// the callback method to listen the command 
// in this case it gets a Twist message and assigns to last_msg private attribute
// we need to implement a real world solution
// for now, it's just to start and learn how to use Object methods as a Subscriber callback
void SampleVelocityModel::listenCMD(const geometry_msgs::Twist msg) {
    cmd = msg;
}