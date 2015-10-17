#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include <thread>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include "SampleSet.hpp"

#include "SampleVelocityModel.hpp"
#include "SampleOdometryModel.hpp"
#include "LikelihoodFieldModel.hpp"
#include "BeamRangeFinderModel.hpp"

class MonteCarloLocalization {

    protected:

        // The set of samples
        SampleSet Xt;

        // the motion model
        SampleMotionModel *motion;

        // the measurement model
        MeasurementModel *measurement;

        // the time to sync Commands and LaserScan
        ros::Time sync;

        // a mutex to avoid multiple starts
        std::mutex mcl_mutex;

        // set a generator engine
        std::default_random_engine generator;

        // the run method is private
        // it can be called only inside the MonteCarloLocalization::start() method
        virtual void run();

    public:


        // basic constructor, it receives private_nh, motion, measurement
        MonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

        // the destructor
        ~MonteCarloLocalization();

        // start a thread
        // it starts a thread that executes the run() method and exits smoothly
        virtual void start();

        // spread all particles
        // avoiding to spread particles at the same time run() method
        virtual void spreadSamples(Map &);

        // resample the entire SampleSet
        virtual void resample();

        // return a copy of the Sample2D
        virtual geometry_msgs::PoseArray getPoseArray();

};

#endif