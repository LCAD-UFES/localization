#include "SampleSet.hpp"

// basic constructor
SampleSet::SampleSet(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) {

    // get the size parameter
    // if not found, it'll be 800'
    private_nh.param<unsigned int>("max_sample_set_size", size, 800);

    // allocate the array of 2D samples
    newSamples();

    // Motion model
    std::string motionModel;
    private_nh.param<std::string>("sample_motion_model", motionModel, "vel");
    if (0 == motionModel.compare("vel")) {
        // the default constructor
        motion = new SampleVelocityModel(nh, private_nh);

    } else if ( 0 == motionModel.compare("odom")) {
        // the default constructor
        motion = new SampleOdometryModel(nh, private_nh);

    } else {
        // Let's make the ODOMETRY_MODEL the default one
        // it's just to the case where we have another Motion Models available
        motion = new SampleOdometryModel(nh, private_nh);

    }

    // badd allocation?
    if (nullptr == motion) {
        throw std::bad_alloc();
    }

    // Measurement Model
    std::string measurementModel;
    private_nh.param<std::string>("measurement_model", measurementModel, "likelyhood");
    if (0 == measurementModel.compare("beam")) {
        // default constructor
        measurement = new BeamRangeFinderModel(nh, private_nh);
    } else if (0 == measurementModel.compare("likelyhood")) {
        // default constructor
        measurement = new LikelyhoodFieldModel(nh, private_nh);
    } else {
        // let's make the BeamRangeFinderModel the default one
        measurement = new BeamRangeFinderModel(nh, private_nh);
    }

    // badd allocation?
    if (nullptr == measurement) {
        throw std::bad_alloc();
    }
}

// destructor
SampleSet::~SampleSet() {

    if (nullptr != motion) {
        delete motion;
    }

    if (nullptr != measurement) {
        delete measurement;
    }

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

void SampleSet::sample() {
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
