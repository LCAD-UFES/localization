#ifndef INJECTION_MONTE_CARLO_LOCALIZATION_H
#define INJECTION_MONTE_CARLO_LOCALIZATION_H

#include "MonteCarloLocalization.hpp"
#include <thread>
#include <iostream>
#include <mutex>
//Monte Carlo Localization with injection of random particle
class InjectionMonteCarloLocalization : public MonteCarloLocalization {

    protected:

        // random amount of particles, it's a percentual
        double random_amount;

        // raondom particles injection rate
        int injection_rate;

        // the resample counter
        int sample_counter;

        // the run method is private
        // it can be called only inside the AugmentedMonteCarloLocalization::start() method
        // override the run method
        virtual void run();

        // ??
        void threadPeso(int inicio, int fim);

        // 
        std::mutex inject_mutex;

    public:


        // Basic constructor
        InjectionMonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

};


#endif
