#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include "CommandVelocity.hpp"
#include "CommandOdometry.hpp"
#include "Laser.hpp"
#include "Map.hpp"
#include "SampleVelocityModel.hpp"
#include "LikelyhoodFieldModel.hpp"

#include "MonteCarloLocalization.hpp"

class ParticleFilter {
    private:

        // ros node handles
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        // subscribers
        ros::Subscriber laser_sub;
        ros::Subscriber cmd_sub;
        ros::Subscriber map_sub;

        // The Command Reader
        std::vector<CommandReader *> cmds;

        // just to mark the TwistStamped seq
        unsigned int cmd_seq;

        // the commands mutex
        std::mutex cmds_mutex;

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

    public:
        // base constructor
        ParticleFilter();
        ~ParticleFilter();

        // callbacks
        // laser received callback
        void laserReceived(const sensor_msgs::LaserScan);

        // the motion command to be used by the Velocity Motion Model
        void commandVelReceived(const geometry_msgs::Twist);
        // the motion command to be used by the Odometry Motion Model
        // we  need to format the correct message and subscribe this callback
        /* TODO */
        void commandOdomReceived(const nav_msgs::Odometry);

        // the map topic
        void readMap(const nav_msgs::OccupancyGrid);

        // run
        void start();
};

#endif