#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include "sample2D.hpp"
#include "gaussianpdf.hpp"

class SampleSet {
    private:

        // number of samples
        unsigned int size;

        // the Sample Motion Model
        SampleMotionModel *motion;

        // the Measurement Model
        MeasurementModel *measure;

        // the set of the pose samples
        Sample2D *samples;

    public:
        // SampletSet constructor
        // it receives the size or amout of samples and
        // two strings that defines the motion and measurement model types
        SampleSet(unsigned int, string motionModel, string measurementModel);
        ~SampleSet();

        // sample the entire set
        void sample();

        // resample the entire set
        void resample();
};

#endif