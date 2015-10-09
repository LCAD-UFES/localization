#include "ParticleFilter.hpp"

// Constructor
ParticleFilter::ParticleFilter() : nh(), private_nh("~"), cmds(), laser(), map() {

    // Motion model
    std::string motionModel;
    private_nh.param<std::string>("sample_motion_model", motionModel, "vel");
    if (0 == motionModel.compare("odom")) {

        // the default constructor
        motion = new SampleOdometryModel(private_nh, &cmds);

    } else {
        // Let's make the Velocity Model the default one
        // the default constructor
        motion = new SampleVelocityModel(private_nh, &cmds);
    }

    // badd allocation?
    if (nullptr == motion) {
        throw std::bad_alloc();
    }

    // Measurement Model
    std::string measurementModel;
    private_nh.param<std::string>("measurement_model", measurementModel, "likelyhood");
    if (0 == measurementModel.compare("beam")) {
        // default constructor
        measurement = new BeamRangeFinderModel(&laser, &map);
    } else {
        // default constructor
        measurement = new LikelyhoodFieldModel(&laser, &map);
    }

    // badd allocation?
    if (nullptr == measurement) {
        throw std::bad_alloc();
    }

    // The MCL object
    mcl = new MonteCarloLocalization(private_nh, motion, measurement);
    if (nullptr == mcl) {
        throw std::bad_alloc();
    }

    // now, with all objects initialized we can start to the subscribe the topics
    // get the laser topic name
    std::string laser_topic;
    private_nh.param<std::string>("laser_scan_topic", laser_topic, "p3dx/laser/scan");
    // subscribe to the laser scan topic
    laser_sub = nh.subscribe(laser_topic, 10, &ParticleFilter::laserReceived, this);

    // get the command topic name
    std::string cmd_topic;
    private_nh.param<std::string>("motion_model_command_topic", cmd_topic, "cmd_vel");

    // subscribe to the command topic
    if (0 == cmd_topic.compare("odom")) {
        cmd_sub = nh.subscribe(cmd_topic, 10, &ParticleFilter::commandOdomReceived, this);
    } else {
        cmd_sub = nh.subscribe(cmd_topic, 10, &ParticleFilter::commandVelReceived, this);
    }

    // get the map topic name
    std::string map_topic;
    private_nh.param<std::string>("map_server_topic", map_topic, "map");
    // subscribe to the map topic
    map_sub = nh.subscribe(map_topic, 1, &ParticleFilter::readMap, this);

}

// Destructor
ParticleFilter::~ParticleFilter() {

    // delete the commands
    while(!cmds.empty()) {
        delete cmds.back();
        bar.pop_back();
    }

    // delete the motion model
    if (nullptr != motion) {
        delete motion;
    }

    // delete the measurement model
    if (nullptr != measurement) {
        delete measurement;
    }

    //delete the MCL
    if (nullptr != mcl) {
        delete mcl;
    }

}

// base constructor

// callbacks
// laser received callback
void ParticleFilter::laserReceived(const nav_msgs::LaserScan msg) {
    // the laser object manages the apropriate mutex
    laser.setScan(msg);
    // starts the MCL
    mcl->start();
}

// the velocity motion command 
void ParticleFilter::commandVelReceived(const geometry_msgs::Twist msg) {
    // creates a new message TwistStamped
    geometry_msgs::TwistStamped stamped;
    // updates the Twist
    stamped.twist = msg;
    // creates a new CommandReader pointer to a CommandVel
    CommandReader *new_cmd = new CommandVel(stamped);
    // push the new command to the command vector
    cmds_mutex.lock();
    cmds.push_back(new_cmd);
    cmds_mutex.unlock();
}

// the odometry motion command
/* TODO */
void ParticleFilter::commandOdomReceived(const nav_msgs::Odometry msg) {}

// the occupancy grid
void ParticleFilter::readMap(const nav_msgs::OccupancyGrid msg) {

    // copy the OccupancyGrid to the Map
    map.setGrid(msg);
}

// run the particle
void ParticleFilter::start() {

    // the asynchronous spinner
    ros::AyncSpinner spinner(1);

    // start spinning
    spinner.start();

    // wait for Control + C
    ros::waitForShutdown();
}