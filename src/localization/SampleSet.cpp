#include "SampleSet.hpp"

// basic constructor
SampleSet::SampleSet(const ros::NodeHandle &private_nh) {
    // get the size parameter
    // if not found, it'll be 800'
    private_nh.param<unsigned int>("max_sample_set_size", size, 800);

    // allocate the array of 2D samples
    newSamples();
}

// destructor
SampleSet::~SampleSet() {
    delete samples;
}

// allocate the samples in memmory
void SampleSet::newSamples() {
    if (0 < size) {
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

void SampleSet::sample(SampleMotionModel* motion, MeasurementModel *measurement) {
    // sample the entire set of particles
    for (int i = 0; i < size; i++) {
        // sample a new pose
        // Pose2D samplePose2D(Pose2D *pose);
        motion->samplePose2D(&samples[i].pose);
        // get a weight
        measurement->getWeight(&samples[i].pose);
    }
}

void SampleSet::resample() {
    /* TODO */
}
