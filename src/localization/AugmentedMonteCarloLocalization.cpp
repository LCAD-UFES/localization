#include "AugmentedMonteCarloLocalization.hpp"

// Basic constructor
AugmentedMonteCarloLocalization::AugmentedMonteCarloLocalization(
                        ros::NodeHandle &private_nh,
                        SampleMotionModel *motion,
                        MeasurementModel *measurement
                     ) : MonteCarloLocalization(private_nh, motion, measurement), w_slow(0), w_fast(0) {

    // get the recovery alpha parameters
    private_nh.param("recovery_alpha_slow", alpha_slow, 0.001);
    private_nh.param("recovery_alpha_fast", alpha_fast, 0.1);

}


// the overrided run method
void AugmentedMonteCarloLocalization::run() {

    // auxiliar variables
    Sample2D *samples = Xt.samples;
    double w_avg = 0.0;

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    motion->update(sync);

    // if the robot moves
    // reset the total weight
    Xt.total_weight = 0.0;

    // SIMPLE SAMPLING
    // iterate over the samples and updates everything
    for (int i = 0; i < Xt.size; i++) {

        // the motion model - passing sample pose by reference
        motion->samplePose2D(&samples[i].pose);

        // the measurement model - passing the Sample2D by pointer
        // the weight is assigned to the sample inside the method
        // it returns the pose weight
        Xt.total_weight  += measurement->getWeight(&samples[i]);

    }

    // updates the average
    w_avg = Xt.total_weight/Xt.size;

    // normalize
    Xt.normalizeWeights();

    // updates the w_slow and w_fast parameters
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

    if (motion->moved){

        // resample the entire SampleSet with random variables option
        resample();

    }

    // usually the MCL returns the Xt sample set
    // what should we do here?
    // let's publish in a convenient topic
    
    // unlock the mutex
    mcl_mutex.unlock();

}

// resample the entire SampleSet with random variables option
void AugmentedMonteCarloLocalization::resample() {

    // auxiliar variables
    double M = 1.0/((double) Xt.size);
    int i = 0;
    double U;

    // shortcut
    Sample2D *samples = Xt.samples;

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

    // create a new Sample2D array
    Sample2D *set = new Sample2D[Xt.size];

    Map *map = measurement->getMap();

    // iterate over the entire SampleSet
    for (int m = 1; m <= Xt.size; m++) {

        if (drand48() < w_diff) {

            // get a random pose
            set[m-1].pose = map->randomPose2D();

            set[m-1].weight = 1.0;

        } else {

            // common low-variance sampler
            U = r + (m-1)*M;

            while (U > c) {

                i = i + 1;
                c += samples[i].weight;

            }

            // copy the x coordinate
            // copy the y coordinate
            // copy the yaw orientation
            set[m-1].pose = samples[i].pose;

            set[m-1].weight = samples[i].weight;

        }

        // updates the new total_weight
        Xt.total_weight += set[m-1].weight;

    }

    // now we have to delete the old array
    delete Xt.samples;

    // assign the new array to this object pointer
    Xt.samples = set;

    // just to be sure...
    set = samples = nullptr;

    // normalize
    Xt.normalizeWeights();

    if (w_diff > 0.0) {

        // reset the w_slow and w_fast parameters
        // it avoids the complete randomness
        w_slow = w_fast = 0.0;

    }

}