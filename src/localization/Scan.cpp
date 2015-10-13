#include "Scan.hpp"

// basic constructor
Scan::Scan() : range_count(0), ranges(nullptr) {}

// copy constructor
Scan::Scan(const Scan &ls) :
                                angle_min(ls.angle_min),
                                angle_max(ls.angle_max),
                                angle_increment(ls.angle_increment),
                                time_increment(ls.time_increment),
                                range_min(ls.range_min),
                                range_max(ls.range_max),
                                range_count(ls.range_count),
                                ranges(nullptr),
                                time(ls.time) {


    // allocate the ranges
    // the Scan destructor will take care of it
    ranges = new float[ls.range_count][2];
    if(nullptr == ranges) {
        throw std::bad_alloc();
    }

    // copy the ranges and angles of the beams
    for (int i = 0; i < range_count; i++) {
        // copy the beam range
        ranges[i][0] = ls.ranges[i][0];
        // copy the beam angle
        ranges[i][1] = ls.ranges[i][1];
    }

}

// destructor
Scan::~Scan() {
    if (nullptr != ranges) {
        delete [] ranges;
    }
}

// update the current Scan with a new ROS sensor_msgs::LaserScan
void Scan::updateScan(const sensor_msgs::LaserScan &msg) {

    // get the amount of beams
    int size = msg.ranges.size();

    // same size?
    if (range_count != size) {

        if (nullptr != ranges) {
            // delete
            delete [] ranges;
        }

        // allocate new space
        ranges = new float[size][2];
        if (nullptr == ranges) {
            throw std::bad_alloc();
        }

    } else if (nullptr == ranges) {

        // allocate new space
        ranges = new float[size][2];
        if (nullptr == ranges) {
            throw std::bad_alloc();
        }

    }

    // how many scans
    range_count = size;


    // copy the basic info
    // start angle of the scan
    angle_min =  msg.angle_min;
    // end angle of the scan
    angle_max = msg.angle_max;
    // angular distance between measurements
    angle_increment = msg.angle_increment;
    // time betwen scans
    time_increment  = msg.time_increment;

    // mininum range value
    range_min = msg.range_min;
    // max range value
    range_max = msg.range_max;

    // range data
    for (int i = 0; i < size; i++) {

        // copy the range value
        if (msg.range_min < msg.ranges[i]) {
            ranges[i][0] = msg.ranges[i];
        } else {
            ranges[i][0] = msg.range_max;
        }

        // the angle value
        ranges[i][1] = msg.angle_min + (i * msg.angle_increment);

    }

    // save the LaserScan timestamp
    time = msg.header.stamp;

}

// copy another Scan
void Scan::copy(const Scan& s) {


    // same size?
    if (range_count != s.range_count) {

        if (nullptr != ranges) {
            // delete
            delete [] ranges;
        }

        // allocate new space
        ranges = new float[s.range_count][2];
        if (nullptr == ranges) {
            throw std::bad_alloc();
        }

    } else if (nullptr == ranges) {

        // allocate new space
        ranges = new float[s.range_count][2];
        if (nullptr == ranges) {
            throw std::bad_alloc();
        }

    }

    // how many scans
    range_count = s.range_count;


    // copy the basic info
    // start angle of the scan
    angle_min =  s.angle_min;
    // end angle of the scan
    angle_max = s.angle_max;
    // angular distance between measurements
    angle_increment = s.angle_increment;
    // time betwen scans
    time_increment  = s.time_increment;

    // mininum range value
    range_min = s.range_min;
    // max range value
    range_max = s.range_max;

    // range data
    for (int i = 0; i < s.range_count; i++) {

        // copy the range value
        ranges[i][0] = s.ranges[i][0];
        // copy the angle value
        ranges[i][1] = s.ranges[i][1];

    }

    // copy the timestamp
    time = s.time;

}