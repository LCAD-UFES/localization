#include "SampleSet.hpp"
#include <cmath>

// basic constructor
SampleSet::SampleSet(const ros::NodeHandle &private_nh) : spreaded(false), samples(nullptr) {

    // get the size parameter
    // if not found, it'll be 800'
    private_nh.param( (std::string) "sample_set_size", size, 800);

    // get the min and max samples
    private_nh.param("min_sample_set_size", min, 100);
    private_nh.param( (std::string) "max_sample_set_size", max, 10000);

    // allocate the array of 2D samples
    newSamples();

}

// destructor
SampleSet::~SampleSet() {
    delete samples;
}

// resampling all particles based on the weight
void SampleSet::resample() {

    // auxiliar variables
    double M = 1.0/size;
    int i = 0;
    double U;

    // our particle weight normalizer
    double normalizer = 1.0/total_weight;

    // a default random generator
    std::default_random_engine generator(std::random_device {} ());

    // a uniform distribution
    // from zero to size - 1, our SampleSet size
    std::uniform_real_distribution<double> uniform(0, M);

    // get a random value
    double r = uniform(generator);

    // get the first weight
    double c = samples[0].weight*normalizer;

    // reset total weight
    total_weight = 0.0;

    // create a new Sample2D array
    Sample2D *set = new Sample2D[size];

    // iterate over the entire SampleSet
    for (int m = 0; m < size; m++) {

        U = r + (m)*M;

        while (U > c) {

            i = i + 1;
            c += samples[i].weight*normalizer;

        }

        // copy the x coordinate
        set[m].pose.v[0] = samples[i].pose.v[0]; 
        // copy the y coordinate
        set[m].pose.v[1] = samples[i].pose.v[1]; 
        // copy the yaw orientation
        set[m].pose.v[2] = samples[i].pose.v[2];

        // copy the weight
        set[m].weight = samples[i].weight;
        // updates the new total_weight
        total_weight += samples[i].weight;

    }

    // now we have to delete the old array
    delete samples;

    // assign the new array to this object pointer
    samples = set;

    // just to be sure...
    set = nullptr;

}

// allocate the samples in memmory
void SampleSet::newSamples() {
    // the sample set size limits
    if (min <= size && size <= max && nullptr == samples) {

        // allocate the memmory
        samples = new Sample2D[size]();
        if (nullptr == samples) {
            throw std::bad_alloc();
        }

        // reset the samples
        resetSamples();

    }

}

// reset all poses to zero and the weigths to 1/size
void SampleSet::resetSamples() {

    if (nullptr != samples) {

        // size must be greater than zero
        double min_w = 1/size;

        for (int i = 0; i < size; i++) {

            samples[i].pose = { 0.0, 0.0, 0.0 };
            samples[i].weight = min_w;

        }

    }

}