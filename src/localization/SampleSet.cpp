#include "SampleSet.hpp"
#include <cmath>
#include <iostream>

// basic constructor
SampleSet::SampleSet(const ros::NodeHandle &private_nh) : spreaded(false), samples(nullptr), old_set(nullptr) {

    // get the size parameter
    // if not found, it'll be 800'
    private_nh.param( (std::string) "sample_set_size", size, 800);

    // get the min and max samples
    private_nh.param("min_sample_set_size", min, 800);
    private_nh.param( (std::string) "max_sample_set_size", max, 20000);

    // allocate the array of 2D samples
    newSamples();

}

// destructor
SampleSet::~SampleSet() {
    delete samples;
}

// normalize the weights
void SampleSet::normalizeWeights() {

    // normalize the weights
    double normalizer;
    if (0 < total_weight) {

        normalizer = 1.0/( (double) total_weight);

        for (int i = 0; i < size; i++) {
//            //verificar os pesos
//            std::cout << "Particula: " << i << std::endl;
//            std::cout << "x: " << samples[i].pose.v[0] << std::endl;
//            std::cout << "y: " << samples[i].pose.v[1] << std::endl;
//            std::cout << "theta: " << samples[i].pose.v[2]<< std::endl;
//            std::cout << "Peso: " << samples[i].weight<< std::endl;


            // normalize
            samples[i].weight *= normalizer;
//            std::cout << "Peso Norm: " << samples[i].weight<< std::endl;
//            std::cout << "-----------------------------" << std::endl;


        }

    } else {

        normalizer = 1.0/size;

        for (int i = 0; i < size; i++) {

            // normalize
            samples[i].weight = normalizer;

        }

    }

}

// allocate the samples in memmory
void SampleSet::newSamples() {

    if (size > max) {
        size = max;
    } else if (size < min) {
        size = min;
    }

    // the sample set size limits
    if (min <= size && size <= max && nullptr == samples) {

        // exception handling
        try {

            // allocate the memmory
            samples = new Sample2D[size]();

        } catch (std::bad_alloc& ba) {

            std::cerr << ba.what() << std::endl;

        }

        // exception handling
        try {

            // allocate the memmory
            old_set = new Sample2D[size]();

        } catch (std::bad_alloc& ba) {

            std::cerr << ba.what() << std::endl;

        }

        // reset the samples
        resetSamples();

    }

}

// reset all poses to zero and the weigths to 1/size
void SampleSet::resetSamples() {

    if (nullptr != samples) {

        // size must be greater than zero
        double min_w = 1.0/((double)size);

        for (int i = 0; i < size; i++) {

            samples[i].pose = { 0.0, 0.0, 0.0 };
            samples[i].weight = min_w;

        }

    }

}
