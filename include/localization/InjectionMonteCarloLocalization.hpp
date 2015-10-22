#ifndef INJECTION_MONTE_CARLO_LOCALIZATION_H
#define INJECTION_MONTE_CARLO_LOCALIZATION_H

#include "MonteCarloLocalization.hpp"

//Monte Carlo Localization with injection of random particle
class InjectionMonteCarloLocalization : public MonteCarloLocalization {

    protected:

        // Parameters
        double number_injections;
        //each "times" samples run injection
        int injection_times;
        Map random_pose;

        // the run method is private
        // it can be called only inside the AugmentedMonteCarloLocalization::start() method
        // override the run method
        virtual void run();

    public:

        // Basic constructor
        InjectionMonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

};


#endif
