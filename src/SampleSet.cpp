#include "SampleSet.hpp"

// We can pass the parameters
SampleSet(unsigned int s, string motionModel, string measurementModel) : size(s) {

    double min_w = 1/s;

    if (motionModel.compare(VELOCITY_MODEL) == 0) {
        motion = new SampleVelocityModel();
    } else if (motion.compare(ODOMETRY_MODEL) == 0) {
        motion = new SampleOdometryModel();
    } else {
        // Let's make the ODOMETRY_MODEL the default one
        // it's just to the case where we have another Motion Models available
        motion = new SampleOdometryModel();
    }

    // badd allocation?
    if (nullptr == motion) {
        throw std::bad_alloc();
    }

    // allocate the array of 2D samples
    samples = new Sample2D[s](min_w);
    if (nullptr == samples) {
        throw std::bac_alloc();
    }

}

// destructor
SampleSet::~SampleSet() {

    if (nullptr != motion) {
        delete motion;
    }

    if (nullptr != measure) {
        delete measure;
    }

    delete samples;
}

void SampleSet::sample(CommandReader *cmd) {
    // sample the entire set of particles
    for (int i = 0; i < size; i++) {
        // sample a new pose
        // Pose2D samplePose2D(CommandVel *cmd, Pose2D *pose);
        motion->sample(cmd, &samples[i].pose);
        // get a weight
        samples[i].weight = measure->sample(cmd, &samples[i].pose);
    }
}

void SampleSet::resample() {
    /* TODO */
}

#endif  