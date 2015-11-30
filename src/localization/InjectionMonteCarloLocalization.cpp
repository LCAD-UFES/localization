#include "InjectionMonteCarloLocalization.hpp"

// Basic constructor
InjectionMonteCarloLocalization::InjectionMonteCarloLocalization(
        ros::NodeHandle &private_nh,
        SampleMotionModel *motion,
        MeasurementModel *measurement
        ) : MonteCarloLocalization(private_nh, motion, measurement), sample_counter(0){

    // get the recovery alpha parameters
    private_nh.param("random_amount_of_samples", random_amount, 0.05);
    private_nh.param("injection_rate", injection_rate, 10);

    // verify the random_amount
    if (0 >= random_amount || random_amount > 1) {
        random_amount = 0.05;
    }

    // the left limit
    limit = Xt.size*(1 - random_amount);

}

// the overrided run method
void InjectionMonteCarloLocalization::run() {

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if(moved){

        // reset the total weight
        Xt.total_weight = 0.0;

        // reset the SampleSet index
        sampleIndex = 0;

        // starts the new threads
        std::vector<std::thread> pool(pool_size);

        // verify if we need random samples
        if (sample_counter < injection_rate) {

            // set the limit to full range
            // in this case we don't need random samples
            limit = Xt.size;

            // update the resample counter
            sample_counter++;

        } else {

            // we need some random samples
            // set the limits
            limit = Xt.size *(1 - random_amount);

            // get some random random samples - just 1 thread
            injectRandomSamples(limit);

            // reset the sample_counter
            sample_counter = 0;

        }

        // spawn each thread
        for (int i = 0; i < pool_size; i++) {

            pool[i] = std::thread(&InjectionMonteCarloLocalization::sample, this);

        }

        // join each thread
        for (int i = 0; i < pool_size; i++) {
            pool[i].join();
        }

        // normalize
        Xt.normalizeWeights();

        // RESAMPLING
        if (resample_rate < resample_counter) {

            // resampling
            resample();

            // reset the resample_counter
            resample_counter = 0;

        } else {

            // increments the resample_counter
            resample_counter++;
        }

    }

    // unlock the mcl mutex
    mcl_mutex.unlock();
}

// inject some random samples
void InjectionMonteCarloLocalization::injectRandomSamples(int index) {

    // shortcut
    Sample2D *samples = Xt.samples;

    Map *m = measurement->getMap();

    // the random sample weight
    double rw = 1.0/(double)Xt.size;

    double t_weight = 0.0;

    // iterates over the last samples and get some random poses
    while (index < Xt.size) {

        // updates the pose
        samples[index].pose = m->randomPose2D();

        // updates the weight
        samples[index].weight = rw;

        // updates the partial weight
        t_weight += rw;

        // next random sample
        index++;

    }

    // save the total weight
    Xt.total_weight += t_weight;

}
