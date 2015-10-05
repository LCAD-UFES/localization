#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include "Sample2D.hpp"
#include "GaussianPDF.hpp"
#include "SampleMotionModel.hpp"
#include "MeasurementModel.hpp"

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

    public:
        // SampletSet constructor
        // it receives the size or amount of samples and
        // two strings that defines the motion and measurement model types
        SampleSet(unsigned int, std::string, std::string);
        // Constructor that receives the motion and measurement models from outside
        SampleSet(unsigned int, SampleMotionModel*, MeasurementModel*);
        ~SampleSet();

        // sample the entire set
        void sample();

        // resample the entire set
        void resample();
};

#endif