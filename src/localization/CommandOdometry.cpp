#include "CommandOdometry.hpp"

CommandOdom::CommandOdom() : poses(), cmds_mutex(), old_pose({0.0, 0.0, 0.0}) {}

void CommandOdom::setNew_pose(const nav_msgs::Odometry &msg){

    //lock the mutex
    cmds_mutex.lock();

    // copy the pose from the Odometry message
    geometry_msgs::PoseStamped p;

    // copy the header
    p.header = msg.header;

    // copy the pose - Can we just drop the Covariance Matrix?
    // the incoming message contains a geometry_msgs::PoseWithCovariance
    // and we are droping the covariance matrix
    p.pose = msg.pose.pose;

    // save the Pose
    poses.push_back(p);

    // unlock the mutex
    cmds_mutex.unlock();

}
// standard vector just to transport the poses
std::vector<Pose2D> CommandOdom::getCommandOdom(const ros::Time &end, bool &move){

    // just a container
    std::vector<Pose2D> commands;

    // lock the mutex
    cmds_mutex.lock();

    // get the last pose
    commands.push_back(old_pose);

    // build the iterators
    // the reverse one

    if (poses.empty()) {
        commands.push_back(old_pose);
    } else {

        geometry_msgs::PoseStamped ps = poses.back();

        poses.clear();
        // copy the command
        Pose2D new_pose = convertToPose2D(ps);

        // push to the commands list
        commands.push_back(new_pose);

        // updates the old_pose
        double x = sqrt(pow(new_pose.v[0] - old_pose.v[0],2) + pow(new_pose.v[1] - old_pose.v[1],2));
        if(x>0.00001 || fabs(mrpt::math::angDistance(new_pose.v[2], old_pose.v[2]))>0.000001){
            old_pose = new_pose;
            move = true;
        }
        else{
            move = false;
        }

    }

    // erase the unnecessary commmands

    // unlock the mutex
    cmds_mutex.unlock();

    // return the commands
    return commands;

}

// geometry_msgs::Pose to our internal representation Pose2D
Pose2D CommandOdom::convertToPose2D(geometry_msgs::PoseStamped p) {

    Pose2D new_pose;

    // copy the x coord
    new_pose.v[0] = p.pose.position.x;
    // copy the y coord
    new_pose.v[1] = p.pose.position.y;

    tf::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);

    // get the quaternion and builds a tf Matrix3x3
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);

    new_pose.v[2] = yaw;

    return new_pose;


}
