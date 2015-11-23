#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>

#include "CommandVelocity.hpp"
#include "CommandOdometry.hpp"
#include "Laser.hpp"
#include "Map.hpp"
#include "SampleVelocityModel.hpp"
#include "LikelihoodFieldModel.hpp"

#include "MonteCarloLocalization.hpp"
#include "AugmentedMonteCarloLocalization.hpp"
#include "InjectionMonteCarloLocalization.hpp"

class ParticleFilter {
    private:

        // ros node handles
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        // subscribers
        ros::Subscriber laser_sub;
        ros::Subscriber cmd_sub;
        ros::Subscriber map_sub;

        // advertises
        ros::Publisher pose_array_pub;

        // the ROS LoopRate
        ros::Rate loop_rate;

        // The CommandVel Reader
        CommandVel cmd_vel;

        // The CommandVel Reader
        CommandOdom cmd_odom;

        // The laser Reader
        Laser laser;

        // the Map reader
        Map map;

        // The Sample Motion Model
        SampleMotionModel *motion;

        // The Measurement Model
        MeasurementModel *measurement;

        // The MCL object
        MonteCarloLocalization *mcl;

        // parameter
        bool spread_samples;

        // broadcaster
        tf::TransformBroadcaster br;

    public:

        // base constructor
        ParticleFilter();
        ~ParticleFilter();

        // callbacks
        // laser received callback
        void laserReceived(const sensor_msgs::LaserScan&);

        // the motion command to be used by the Velocity Motion Model
        void commandVelReceived(const geometry_msgs::Twist&);

        // the motion command to be used by the Odometry Motion Model
        // we  need to format the correct message and subscribe this callback
        /* TODO */
        void commandOdomReceived(const nav_msgs::Odometry&);

        // the map topic
        void readMap(const nav_msgs::OccupancyGrid&);

        // publish PoseArray to /pose_array topic
        void publishPoseArray();

        // broadcaster PoseArray to /pose_array topic
        void broadcastMeanPose();

        // run
        void start();

        // update
};

#endif
