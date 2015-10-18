#include "CommandOdometry.hpp"

CommandOdom::CommandOdom() : cmds_mutex(), new_pose(), old_pose() {}

     void CommandOdom::setNew_pose(const nav_msgs::Odometry &msg){
         //lock the mutex
         cmds_mutex.lock();
        //Odometry to Pose2D
        new_pose.v[0] = msg.pose.pose.position.x;
        new_pose.v[1] = msg.pose.pose.position.y;
        tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        new_pose.v[3] = yaw;

        // unlock the mutex
        cmds_mutex.unlock();
}
     void CommandOdom::setOld_pose(Pose2D odom){
         old_pose = odom;
     }
        //
     std::vector<Pose2D> CommandOdom::getCommandOdom(){
         // lock the mutex
         cmds_mutex.lock();

         std::vector<Pose2D> commands;
         commands.push_back(old_pose);
         commands.push_back(new_pose);

         // unlock the mutex
         cmds_mutex.unlock();

     }


