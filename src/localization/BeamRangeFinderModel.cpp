#include "BeamRangeFinderModel.hpp"
#include <math.h>

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {
    // get the z_hit parameter
    private_nh.param("likelihood_z_hit", z_hit, 0.9);
    // get the z_short parameter
    private_nh.param("likelihood_z_short", z_short, 0.9);
    // get the z_max parameter
    private_nh.param("likelihood_z_max", z_max, 0.05);
    // get the z_rand parameter
    private_nh.param("likelihood_z_rand", z_rand, 0.05);
    // get the sigma_hit parameter
    private_nh.param("likelihood_sigma_hit", sigma_hit, 0.2);
    private_nh.param("likelihood_lambda_short", lambda_short, 0.2);

    // get the max_beams parameter, see MeasurementModel base class
    private_nh.param("laser_max_beams", max_beams, 60);

    //pre-computing a few things

    sigma_hit2 = sigma_hit*sigma_hit;
    part_gaussian = 1/sqrt(2*M_PI*sigma_hit2);
    z_random_max = z_rand/30.0;

}

//
double BeamRangeFinderModel::getWeight(Sample2D *sample) {
    /* TODO */
}

// update the LaserScan
ros::Time BeamRangeFinderModel::update() {
    /* TODO */
    return ros::Time::now();
}
