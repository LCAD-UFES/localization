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
std::vector<Pose2D> CommandOdom::getCommandOdom(const ros::Time &end){

    // just a container
    std::vector<Pose2D> commands;

    // lock the mutex
    cmds_mutex.lock();

    // get the last pose
    commands.push_back(old_pose);

    // build the iterators
    // the reverse one
    std::list<geometry_msgs::PoseStamped>::reverse_iterator rit(poses.rbegin());
    // the first element
    std::list<geometry_msgs::PoseStamped>::iterator it(poses.begin());

    // iterate the poses List and get the last command before the LaserScan
    while(rit->header.stamp > end && rit->header.stamp != it->header.stamp) {

        // it's a plus plus sign (++) but this a reverse iterator!! Under the hood it's looks like a (--)
        rit++;

    }

    // copy the command
    Pose2D new_pose = convertToPose2D(rit);

    // push to the commands list
    commands.push_back(new_pose);

    // updates the old_pose
    old_pose = new_pose;

    // slice the list
    std::list<geometry_msgs::PoseStamped>::iterator it2 = poses.begin();
    while(it2->header.stamp < rit->header.stamp) {
        it2++;
    }
    poses.erase(it, it2);

    // push the updated old_pose
    commands.push_back(old_pose);

    // unlock the mutex
    cmds_mutex.unlock();

    // return the commands
    return commands;

}

// geometry_msgs::Pose to our internal representation Pose2D
Pose2D CommandOdom::convertToPose2D(geometry_msgs::PoseStamped p) {

    Pose2D new_pose;

    // lock the mutex
    cmds_mutex.lock();
    
    // copy the x coord
    new_pose.v[0] = p.pose.position.x;
    // copy the y coord
    new_pose.v[1] = p.pose.position.y;

    tf::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);

    // get the quaternion and builds a tf Matrix3x3
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);

    new_pose.v[3] = yaw;

    // unlock the mutex
    cmds_mutex.unlock();

    return new_pose;


}