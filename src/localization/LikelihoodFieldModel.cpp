#include "LikelihoodFieldModel.hpp"

// Basic constructor
LikelihoodFieldModel::LikelihoodFieldModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {

    // get the z_hit parameter
    private_nh.param("likelihood_z_hit", z_hit, 0.97);
    // get the z_max parameter
    private_nh.param("likelihood_z_max", z_max, 0.02);
    // get the z_rand parameter
    private_nh.param("likelihood_z_rand", z_rand, 0.01);

    // get the sigma_hit parameter == standard deviation
    private_nh.param("likelihood_sigma_hit", sigma_hit, 0.005);

    // get the max_beams parameter, see MeasurementModel base class
    private_nh.param("laser_max_beams", max_beams, 180);

    // verify sigma_hit
    if (0 >= sigma_hit) {
        sigma_hit = 0.005;
    }

    // the inverse standard deviation
    sigma_hit_inverse = 1.0/sigma_hit;

    norm = 1.0/sqrt(2*M_PI);

    // get the normalizer parameter
    private_nh.param("sample_set_size", normalizer, 1200.0);

    normalizer = 1.0/normalizer;

}

// assigns a weight to to all particles/samples
double LikelihoodFieldModel::getWeight(Sample2D *sample) {

    // auxiliar variables
    double p = 0.0;
    double dist;
    double obs_range;
    double obs_bearing;
    // the endpoint of the beam
    double x, y;

    // the endpoint in the MapGrid coords
    int x_map, y_map;

    // shortcut
    double *pose = sample->pose.v;

    // if the current pose is inside a obstacle...
    if (!grid.validPose(pose[0], pose[1])) {

        // just reducing the probability
        sample->weight *= 0.0;

        return sample->weight;

    }

    // iterate over the scans
    // we have 60
    for (int i = 0; i < ls_scan.size; i += step) {

        // copy the range and the angle
        obs_range = ls_scan.ranges[i][0];
        obs_bearing = ls_scan.ranges[i][1];

        // consider only ranges below range_max and check for NaN
        if (obs_range < ls_scan.range_max) {

            // get the x and y observation coordinates
            // without tf, we assume the laser is in the center of mass
            x = pose[0] + obs_range * cos(pose[2] + obs_bearing);
            y = pose[1] + obs_range * sin(pose[2] + obs_bearing);

            // the recommended one in a real life situation
            // x = pose[0] + x_s*cos(pose[2]) - y_s*sin(pose[2]) + obs_range * cos(pose[2] + obs_bearing);
            // y = pose[0] + y_s*cos(pose[2]) + x_s*sin(pose[2]) + obs_range * sin(pose[2] + obs_bearing);

            // Convert from world coords to map coords
            x_map = std::floor((x - grid.origin_x)/grid.scale + 0.5) + (grid.width >> 1);
            y_map = std::floor((y - grid.origin_y)/grid.scale + 0.5) + (grid.height >> 1);

            // get the distance
            // verify the bounds
            if (x_map > grid.width || y_map > grid.height) {

                dist = grid.max_occ_dist;

            } else {

                // get the pre-computed value
                dist = grid.getMinDistance(x_map, y_map);

            }

            // updates the dist since we are normalizing our gaussian distribution
            // dist = (dist - mean)/stddev, but mean == 0, so:
            dist *= sigma_hit_inverse;

            // we got some pre-computed work
            p += (z_hit*(norm*std::exp(-0.5*dist*dist)) + z_random_max);

        }

    }

    // ???
    sample->weight = p;

    // save the weight
    return sample->weight;

}

// update the LaserScan
ros::Time LikelihoodFieldModel::update() {

    // update the laser
    laser->getScan(&ls_scan);

    // update the range z_random_max
    z_random_max = z_rand/ls_scan.range_max;

    // update the step
    step = ls_scan.size/(max_beams -1);

    // being cautious
    if (1 > step) {
        step = 1;
    }

    // update the GridMap if necessary
    map->getGridMap(&grid);

    // the laser scan time, used to sync everything
    return ls_scan.time;

}
