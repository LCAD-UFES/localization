#include "AugmentedMonteCarloLocalization.hpp"

// Basic constructor
AugmentedMonteCarloLocalization::AugmentedMonteCarloLocalization(
                        ros::NodeHandle &private_nh,
                        SampleMotionModel *motion,
                        MeasurementModel *measurement
                     ) : MonteCarloLocalization(private_nh, motion, measurement), w_slow(0), w_fast(0) {

    // get the recovery alpha parameters
    private_nh.param("recovery_alpha_slow", alpha_slow, 0.001);
    private_nh.param("recovery_alpha_fast", alpha_fast, 0.5);

    // set the limit to Xt.size
    limit = Xt.size;

}


// the overrided run method
void AugmentedMonteCarloLocalization::run() {

    // the weight average
    double w_avg = 0.0;

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if (moved) {

        // reset the total weight
        Xt.total_weight = 0.0;

        // reset the SampleSet index
        sampleIndex = 0;

        // starts the new threads
        std::vector<std::thread> pool(pool_size);

        // spawn each thread
        for (int k = 0; k < pool_size; k++) {

            // see MonteCarloLocalization::sample() implementation
            pool[k] = std::thread(&AugmentedMonteCarloLocalization::sample, this);

        }

        // join each thread
        for (int k = 0; k < pool_size; k++) {
            pool[k].join();
        }

        // updates the average
        w_avg = Xt.total_weight/(double)Xt.size;

        // normalize
        Xt.normalizeWeights();

        // updates the w_slow and w_fast parameters
        if(0.0 == w_slow) {
            w_slow = w_avg;
        } else {
            w_slow += alpha_slow*(w_avg - w_slow);
        }

        if (0.0 == w_fast) {
            w_fast = w_avg;
        } else {
            w_fast += alpha_fast*(w_avg - w_fast);
        }

        // resample the entire SampleSet with random variables option
        resample();

    }

    // unlock the mutex
    mcl_mutex.unlock();

}

// resample the entire SampleSet with random variables option
void AugmentedMonteCarloLocalization::resample() {

    // auxiliar variables
    double M = 1.0/((double) Xt.size);
    int i = 0;
    double U;

    double ux = 0;
    double uy = 0;
    double utheta = 0;
    double x_component = 0;
    double y_component = 0;

    // shortcuts
    Sample2D *samples = Xt.samples;
    Sample2D *set = Xt.old_set;

    double w_diff = 1.0 - w_fast/w_slow;

    if (0.0 > w_diff) {

        w_diff = 0.0;

    }

    // a uniform distribution
    // from zero to size - 1, our SampleSet size
    std::uniform_real_distribution<double> uniform(0, M);

    // get a random value
    double r = uniform(generator);

    // get the first weight
    double c = samples[0].weight;

    // reset total weight
    Xt.total_weight = 0.0;

    Map *map = measurement->getMap();

    // iterate over the entire SampleSet
    for (int m = 0; m < Xt.size; m++) {

        if (drand48() < w_diff) {

            // get a random pose
            set[m].pose = map->randomPose2D();

        } else {

            // common low-variance sampler
            U = r + (m)*M;

            while (U > c) {

                i = i + 1;
                c += samples[i].weight;

            }

            // copy the x coordinate
            // copy the y coordinate
            // copy the yaw orientation
            set[m].pose = samples[i].pose;

        }

        // the x coordinate mean
        ux += set[m].pose.v[0];

        // the y coordinate mean
        uy += set[m].pose.v[1];

        // the x_component
        x_component += std::cos(samples[m].pose.v[2]);

        // the y_component
        y_component += std::sin(samples[m].pose.v[2]);

    }

    // store the mean pose
    // update the coordinates and the orientation
    Xt.mean_pose.v[0] = ux*M;
    Xt.mean_pose.v[1] = uy*M;
    Xt.mean_pose.v[2] = std::atan2(y_component*M, x_component*M);


    // swap the sets
    Sample2D *temp = Xt.samples;

    // swap the new and old set
    Xt.samples = Xt.old_set;
    Xt.old_set = temp;

    // just to be sure...
    temp = nullptr;
    samples = nullptr;

    if (w_diff > 0.0) {

        // reset the w_slow and w_fast parameters
        // it avoids the complete randomness
        w_slow = w_fast = 0.0;

    }

}
