// #include <tf/transform_broadcaster.h>
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

    //
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

    Pose2D best_pose;

    double ux = 1;
    double uy = 1;
    double utheta = 1;
    double x_component;
    double y_component;

    // normalize the weights
    double normalizer = 1.0/((double) total_weight);

    std::cout << "Total Weight: " << total_weight << std::endl;

    for (int i = 0; i < size; i++) {

        // normalize
        samples[i].weight *= normalizer;

        // x coordinate mean
        ux += samples[i].pose.v[0];
        // y coordinate mean
        uy += samples[i].pose.v[1];

        //
        x_component += std::cos(samples[i].pose.v[2]);
        //
        y_component += std::sin(samples[i].pose.v[2]);

    }
    // get the ux
    best_pose.v[0] = ux/size;
    best_pose.v[1] = uy/size;
    best_pose.v[2] = std::atan2(y_component/size, x_component/size);

/*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(best_pose.v[0], best_pose.v[1], 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, best_pose.v[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "localizer"));*/

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
