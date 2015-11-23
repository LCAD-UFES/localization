#include "LikelihoodFieldModel.hpp"

// Basic constructor
LikelihoodFieldModel::LikelihoodFieldModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {

    // get the z_hit parameter
    private_nh.param("likelihood_z_hit", z_hit, 0.8);
    // get the z_max parameter
    private_nh.param("likelihood_z_max", z_max, 0.05);
    // get the z_rand parameter
    private_nh.param("likelihood_z_rand", z_rand, 0.05);
    // get the sigma_hit parameter
    private_nh.param("likelihood_sigma_hit", sigma_hit, 0.1);

    // get the max_beams parameter, see MeasurementModel base class
    private_nh.param("laser_max_beams", max_beams, 60);

    // let's do some pre-work
    sigma_hit2 = sigma_hit*sigma_hit;
    //
    sigma_hit_den = -1.0/(2*sigma_hit2);
    //
    prob = 1.0/(sigma_hit*sqrt(2*M_PI));
    //
    z_random_max = z_rand/30.0;

}

// assigns a weight to to all particles/samples
double LikelihoodFieldModel::getWeight(Sample2D *sample) {

    // auxiliar variables
    double p = 1.0;
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

            // we got some pre-computed work, let the probability be
            // p = p*(z_hit*(1/sqrt(2*PI*sigma_hit*sigma_hit))*exp(-(x*x)/2*sigma_hit*sigma_hit) + z_rand/z_max);
            // we got the sigma_hit^2 : sigma_hit2 = sigma_hit*sigma_hit;
            // we pre-computed the sigma_hit_den inside exponential = -1.0/(2*sigma_hit2);
            // and pre-computing the left side:
            // prob = 1/(sqrt((2*M_PI)*sigma_hit2))
            // and finally z_random_max = z_rand/z_max
            // let's hope no bugs here = )
            p += z_hit*(exp(dist*dist*sigma_hit_den)) + z_random_max;

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
