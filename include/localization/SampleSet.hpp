#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include "Sample2D.hpp"
#include "GaussianPDF.hpp"
#include "SampleVelocityModel.hpp"
#include "SampleOdometryModel.hpp"
#include "BeamRangeFinderModel.hpp"
#include "LikelyhoodFieldModel.hpp"

#include "ros/ros.h"

class SampleSet {
    private:

        // number of samples
        unsigned int size;

        // the Sample Motion Model
        SampleMotionModel *motion;

        // the Measurement Model
        MeasurementModel *measurement;

        // the set of the pose samples
        Sample2D *samples;

        // private function just to allocate the samples
        void newSamples();

    public:
        // SampletSet basic constructor
        SampleSet(const ros::NodeHandle&);
        ~SampleSet();

        // sample the entire set
        void sample();

        // resample the entire set
        void resample();

        // clear the entire set
        void resetSamples();

};

#endif