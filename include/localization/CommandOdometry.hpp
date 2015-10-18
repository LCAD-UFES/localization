#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include "Pose2D.hpp"
#include <ros/ros.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "wrap2pi.h"

class CommandOdom {

    private:
        // the old and new poses are set by the ParticleFilter callback method
        Pose2D old_pose, new_pose;
        std::mutex cmds_mutex;

    public:
        CommandOdom();
        //receive a new message
        void setNew_pose(const nav_msgs::Odometry &msg);
        //update old_pose
        void setOld_pose(Pose2D odom);
        //return old and atual odom (comand ut(xt-1, xt))
        std::vector<Pose2D> getCommandOdom();
    //

};
#endif
