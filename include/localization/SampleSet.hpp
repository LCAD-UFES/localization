#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include "ros/ros.h"

#include "Sample2D.hpp"

#include "SampleVelocityModel.hpp"
#include "SampleOdometryModel.hpp"
#include "BeamRangeFinderModel.hpp"
#include "LikelyhoodFieldModel.hpp"


class SampleSet {
    private:

        // number of samples or particles
        unsigned int size;

        // the set of the pose samples
        Sample2D *samples;

        // private function just to allocate the samples
        void newSamples();

    public:
        // SampletSet basic constructor
        SampleSet(const ros::NodeHandle&);
        ~SampleSet();

        // sample the entire set
        void sample(SampleMotionModel *, MeasurementModel *);

        // resample the entire set
        void resample();

        // clear the entire set
        void resetSamples();

};

#endif