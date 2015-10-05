#include "SampleSet.hpp"

// We can pass the parameters
SampleSet(unsigned int s, std::string motionModel, std::string measurementModel) : size(s) {

    double min_w = 1/s;

    // allocate the array of 2D samples
    samples = new Sample2D[s](min_w);
    if (nullptr == samples) {
        throw std::bac_alloc();
    }
    // Motion model
    if (0 == motionModel.compare("vel")) {
        // the default constructor
        motion = new SampleVelocityModel();
    } else if ( 0 == motion.compare("odom")) {
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
SampleSet(unsigned int s, SampleMotionModel *motionModel, MeasurementModel *measurementModel) {

    double min_w = 1/s;

    // allocate the array of 2D samples
    samples = new Sample2D[s](min_w);
    if (nullptr == samples) {
        throw std::bac_alloc();
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

    if (nullptr != measure) {
        delete measure;
    }

    delete samples;
}

// 
void SampleSet::sample(CommandReader *cmd) {
    // sample the entire set of particles
    for (int i = 0; i < size; i++) {
        // sample a new pose
        // Pose2D samplePose2D(CommandVel *cmd, Pose2D *pose);
        motion->sample(cmd, &samples[i].pose);
        // get a weight
        measure->sample(cmd, &samples[i].pose);
    }
}

void SampleSet::resample() {
    /* TODO */
}