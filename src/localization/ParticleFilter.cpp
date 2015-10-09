#include "ParticleFilter.hpp"

// Constructor
ParticleFilter::ParticleFilter() : nh(), private_nh("~"), cmds(), laser(), map() {

    // Motion model
    std::string motionModel;
    private_nh.param<std::string>("sample_motion_model", motionModel, "vel");
    if (0 == motionModel.compare("vel")) {
        // the default constructor
        motion = new SampleVelocityModel(private_nh, &cmds);

    } else if ( 0 == motionModel.compare("odom")) {
        // the default constructor
        motion = new SampleOdometryModel(private_nh, &cmds);

    } else {
        // Let's make the ODOMETRY_MODEL the default one
        // it's just to the case where we have another Motion Models available
        motion = new SampleOdometryModel(private_nh, &cmds);

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
        measurement = new BeamRangeFinderModel(private_nh, &laser, &map);
    } else if (0 == measurementModel.compare("likelyhood")) {
        // default constructor
        measurement = new LikelyhoodFieldModel(private_nh, &laser, &map);
    } else {
        // let's make the BeamRangeFinderModel the default one
        measurement = new BeamRangeFinderModel(private_nh, &laser, &map);
    }

    // badd allocation?
    if (nullptr == measurement) {
        throw std::bad_alloc();
    }

    // The MCL object
    mcl = new MonteCarloLocalization(private_nh, motion, measurement);

    // subscribe to the laser scan topic
    laser_sub = nh.subscribe("p3dx/laser/scan", 1, &ParticleFilter::laserReceived, this);

    // subscribe to the command topic
    cmd_sub = nh.subscribe("cmd_vel", 10, &ParticleFilter::commandReceived, this);

    // subscribe to the map topic
    map_sub = nh.subscribe("map", 1, &ParticleFilter::readMap, this);

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
    // now we should signal the MCL object to run
    mcl->start();
}

// the motion command 
void ParticleFilter::commandReceived(const geometry_msgs::TwistStamped msg) {

    // push the new message to the command vector
    cmds.push_back(msg);
}

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