#include "LikelihoodFieldModel.hpp"

// Basic constructor
LikelihoodFieldModel::LikelihoodFieldModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {

    // get the zhit parameter
    private_nh.param("likelihood_z_hit", z_hit, 0.5);
    private_nh.param("likelihood_z_max", z_max, 0.25);
    private_nh.param("likelihood_z_rand", z_rand, 0.25);

    // get the max_beams parameter, see MeasurementModel base class
    private_nh.param("laser_max_beams", max_beams, 60);

}

// assigns a weight to to all particles/samples
void LikelihoodFieldModel::getWeight(Sample2D *sample) {

    // auxiliar variables
    double p = 1.0;
    double dist;
    double obs_range;
    double obs_bearing;
    // the endpoint of the beam
    double x, y;

    // shortcut
    double *pose = sample->pose.v;

    // iterate over the scans
    // we have 60 
    for (int i = 0; i < ls_scan.range_count ; i += step) {

        // copy the range and the angle
        obs_range = ls_scan.ranges[i][0];
        obs_bearing = ls_scan.ranges[i][1];

        // consider only ranges below range_max and check for NaN
        if (obs_range < ls_scan.range_max && obs_range != obs_range) {

            // get the x and y observation coordinates
            // without tf, we assume the laser is in the center of mass
            x = pose[0] + obs_range * cos(pose[2] + obs_bearing);
            y = pose[1] + obs_range * sin(pose[2] + obs_bearing);
            // the recommended one in a real life situation
            // x = pose[0] + x_s*cos(pose[2]) - y_s*sin(pose[2]) + obs_range * cos(pose[2] + obs_bearing);
            // y = pose[0] + y_s*cos(pose[2]) + x_s*sin(pose[2]) + obs_range * sin(pose[2] + obs_bearing);

            dist = 0.0;
        }
    }

    
    sample->weight = 0.025;
}

// get the map pointer
Map* LikelihoodFieldModel::getMap() {
    return map;
}

// update the LaserScan
ros::Time LikelihoodFieldModel::updateLaser() {

    // the Laser object manages the mutex
    laser->getScan(&ls_scan);

    // update the step
    step = ls_scan.range_count/(max_beams -1);
    // being cautious
    if (1 > step) {
        step = 1;
    }

    // pre-computing z_hit
    
    return ls_scan.time;
}