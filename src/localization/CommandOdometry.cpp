#include "CommandOdometry.hpp"

CommandOdom::CommandOdom() : poses(), cmds_mutex(), old_pose({0.0, 0.0, 0.0}) {}

void CommandOdom::setNew_pose(const nav_msgs::Odometry &msg){

    //lock the mutex
    cmds_mutex.lock();

    // save the Pose
    poses.push(msg.pose.pose);

    // unlock the mutex
    cmds_mutex.unlock();

}
// standar vector just to transport the poses
std::vector<Pose2D> CommandOdom::getCommandOdom(ros::Time &end){

    // just a container
    std::vector<Pose2D> commands;

    // lock the mutex
    cmds_mutex.lock();

    // get the last pose
    commands.push_back(old_pose);

    // iterate over the poses vector and get the last Odom command before the LaserScan
    int size = poses.size() - 1;
    if (1 > size) {

        // just 1 odom command, so..
        // update the old pose to the next iteration
        old_pose = convertToPose2D(poses[0].pose);

    } else {

        // 1 or more commands
        // verify the last command
        if (poses[size].header.stamp <= end) {

            // the last command is a valid one
            // update the old pose to the next iteration
            old_pose = convertToPose2D(poses[0].pose);

        } else {

            // the valid command is somewhere in the middle
            for (int i = 0; i < size; i++) {

                // verify the next one
                if (poses[i+1].header.stamp > end) {

                    // get the current
                    // update the old pose to the next iteration
                    old_pose = convertToPose2D(poses[0].pose);

                }

            }

        }

    } else {

        // ERROR, this should not happen!
    }


    // push the updated old_pose
    commands.push_back(old_pose);

    // unlock the mutex
    cmds_mutex.unlock();

    // return the commands
    return commands;

}

// geometry_msgs::Pose to our internal representation Pose2D
Pose2D CommandOdom::convertToPose2D(geometry_msgs::Pose p) {

    Pose2D new_pose;

    // copy the x coord
    new_pose.v[0] = msg.pose.pose.position.x;
    // copy the y coord
    new_pose.v[1] = msg.pose.pose.position.y;

    // get the quaternion and builds a tf Matrix3x3
    tf::Matrix3x3 m(p.orientation);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);

    new_pose.v[3] = yaw;

    return new_pose;

}