#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include <mutex>
#include "MeasurementModel.hpp"
#include "occupancy_grid_utils/ray_tracer.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/LaserScan.h"


class BeamRangeFinderModel : public MeasurementModel {

    private:
        //BeamRangeFinderModel parameters
        double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short;
        //aux
        double ztk, ztk_star, delta_ztk, part_gaussian, z_random_max, sigma_hit2;

        // our internal copy
        sensor_msgs::LaserScan laser_update;
        //mutex
        std::mutex beam_mutex;

    public:
        // the beam model
        BeamRangeFinderModel(ros::NodeHandle&, Laser*, Map* );

        // base class abstract method implementation
        virtual double getWeight(Sample2D *);

        // update the LaserScan
        virtual ros::Time update();

        //
        const geometry_msgs::Pose convertToPose(Sample2D *p);

        // occupancy grid
        nav_msgs::OccupancyGrid map_update;

};
// i is the collum and j is the row :-/
#define MAP_INDEX2(i, j) ((i) + (j) * (map_update.info.width/2))

#endif
