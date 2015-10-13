#ifndef SAMPLE_SET_H
#define SAMPLE_SET_H

#include <ros/ros.h>

#include "Sample2D.hpp"
#include "Map.hpp"

class SampleSet {
    private:
        // spreaded?
        bool spreaded;
        // private function just to allocate the samples
        void newSamples();

    public:
        // SampletSet basic constructor
        SampleSet(const ros::NodeHandle&);
        ~SampleSet();

        // clear the entire set
        void resetSamples();

        // uniform random distribution
        void uniformSpread(Map&);

        // the attributes
        // actual number of samples/particles
        int size;

        // the set of the pose samples
        Sample2D *samples;

        // the min parameter
        int min;

        // the max samples parameter
        int max;

};

#endif