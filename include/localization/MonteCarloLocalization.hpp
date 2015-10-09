#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include <thread>
#include <mutex>
#include <string>

#include "ros/ros.h"

#include "SampleSet.hpp"

class MonteCarloLocalization {
    private:
        // The set of samples
        SampleSet Xt;
        // the motion model
        SampleMotionModel *motion;
        // the measurement model
        MeasurementModel *measurement;

        // a mutex to avoid multiple starts
        std::mutex mcl_mutex;

    public:
        // basic constructor, it receives private_nh, motion, measurement
        MonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

        // the destructor
        ~MonteCarloLocalization();

        // the run method
        void run();

        // start a thread
        // it starts a thread that executes the run() method and exits smoothly
        void start();

};

#endif