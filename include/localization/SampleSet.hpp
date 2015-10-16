#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include <ros/ros.h>

#include "Sample2D.hpp"

class SampleSet {
    private:
        // private function just to allocate the samples
        void newSamples();

    public:
        // SampletSet basic constructor
        SampleSet(const ros::NodeHandle&);
        ~SampleSet();

        // resample all particles based on the pose and weight
        void resample();

        // clear the entire set
        void resetSamples();

        // the attributes
        // actual number of samples/particles
        int size;

        // the set of the pose samples
        Sample2D *samples;

        // the total weight
        double total_weight;

        // the min parameter
        int min;

        // the max samples parameter
        int max;

        // spreaded?
        bool spreaded;


};

#endif