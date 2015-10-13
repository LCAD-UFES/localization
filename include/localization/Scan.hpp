#ifndef SCAN_H
#define SCAN_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Scan {

    public:

        // start angle of the scan
        float angle_min;
        // end angle of the scan
        float angle_max;
        // angular distance between measurements
        float angle_increment;
        // time betwen scans
        float time_increment;

        // mininum range value
        float range_min;
        // max range value
        float range_max;

        // range data
        float (*ranges)[2];

        // custom parameters
        // the number of samples
        int range_count;

        // the LaserScan time, just to sync with the commands
        ros::Time time;

        // basic constructor
        Scan();

        // copy constructor
        Scan(const Scan&);

        // destructor
        ~Scan();

        // update the current Scan with a new ROS sensor_msgs::LaserScan
        void updateScan(const sensor_msgs::LaserScan&);

        // copy another Scan
        void copy(const Scan&);

};
#endif