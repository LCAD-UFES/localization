#include "SampleSet.hpp"

// We can pass the parameters
SampleSet::SampleSet(unsigned int s, std::string motionModel, std::string measurementModel) : size(s) {

    // allocate the array of 2D samples
    samples = new Sample2D[s]();
    if (nullptr == samples) {
        throw std::bad_alloc();
    }
    // clear the samples
    clearSamples();

    // Motion model
    if (0 == motionModel.compare("vel")) {
        // the default constructor
        motion = new SampleVelocityModel();

    } else if ( 0 == motionModel.compare("odom")) {
        // the default constructor
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

    // Measurement Model
    if (0 == measurementModel.compare("beam")) {
        // default constructor
        measurement = new BeamRangeFinderModel();
    } else if (0 == measurementModel.compare("likely")) {
        // default constructor
        measurement = new LikelyhoodFieldModel();
    } else {
        // let's make the BeamRangeFinderModel the default one
        measurement = new BeamRangeFinderModel();
    }

    // badd allocation?
    if (nullptr == measurement) {
        throw std::bad_alloc();
    }
}

// Constructor that receives the motion and measurement models from outside
SampleSet::SampleSet(unsigned int s, SampleMotionModel *motionModel, MeasurementModel *measurementModel) {

    double min_w = 1/s;

    // allocate the array of 2D samples
    samples = new Sample2D[s]();
    if (nullptr == samples) {
        throw std::bad_alloc();
    }

    // the SampleMotionModel, SampleVelocityModel or SampleOdometryModel
    motion = motionModel;
    // the MeasurementModel, BeamRangeFinderModel or LikelyhoodFieldModel
    measurement = measurementModel;

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

// 
void SampleSet::sample(CommandReader *cmd) {
    // sample the entire set of particles
    for (int i = 0; i < size; i++) {
        // sample a new pose
        // Pose2D samplePose2D(CommandVel *cmd, Pose2D *pose);
        motion->samplePose2D(&samples[i].pose);
        // get a weight
        measurement->getWeight(&samples[i].pose);
    }
}

void SampleSet::resample() {
    /* TODO */
}

void SampleSet::clearSamples() {
    if (nullptr != samples) {
        // size must be greater than zero
        double min_w = 1/size;

        for (int i = 0; i < size; i++) {
            samples[i].pose = { 0.0, 0.0, 0.0 };
            samples[i].weight = min_w;
        }
    }
}