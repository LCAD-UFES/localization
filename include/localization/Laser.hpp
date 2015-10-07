#ifndef LASER_H
#define LASER_H

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class Laser {
    private:
        // the processed scan data
        sensor_msgs::PointCloud *cloud;

        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

    public:
        sensor_msgs::PointCloud* toCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif