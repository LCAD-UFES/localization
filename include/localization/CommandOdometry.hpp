#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include <mutex>
#include <list>
#include <vector>

#include "Pose2D.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_datatypes.h"

class CommandOdom {

    private:

        // the command queue
        std::list<geometry_msgs::PoseStamped> poses;

        // the old and new poses are set by the ParticleFilter callback method
        Pose2D old_pose;

        // mutex used to lock the entire CommandOdom object
        std::mutex cmds_mutex;

    public:

        // basic constructor
        CommandOdom();

        //receive a new message
        void setNew_pose(const nav_msgs::Odometry &msg);

        //return old and atual poses from odom (comand ut(xt-1, xt))
        std::vector<Pose2D> getCommandOdom(const ros::Time&);

        // geometry_msgs::Pose to our internal representation Pose2D
        Pose2D convertToPose2D(geometry_msgs::PoseStamped p);

};
#endif
