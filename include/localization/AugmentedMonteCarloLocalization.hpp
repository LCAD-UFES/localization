#ifndef AUGMENTED_MONTE_CARLO_LOCALIZATION_H
#define AUGMENTED_MONTE_CARLO_LOCALIZATION_H

#include "MonteCarloLocalization.hpp"

class AugmentedMonteCarloLocalization : public MonteCarloLocalization {

    protected:

        // Parameters
        double w_slow;
        double w_fast;

        //
        double alpha_slow;
        double alpha_fast;

        // the number of threads inside our pool
        int thread_pool_size;

        // the thread pool size limit
        int thread_pool_size_limit;

        // the run method is private
        // it can be called only inside the AugmentedMonteCarloLocalization::start() method
        // override the run method
        virtual void run();

        // override the resample method
        virtual void resample();

    public:

        // Basic constructor
        AugmentedMonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

};


#endif