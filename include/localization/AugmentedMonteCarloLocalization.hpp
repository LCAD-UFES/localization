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

        // the resample counter
        int resample_counter;

        // the run method is private
        // it can be called only inside the AugmentedMonteCarloLocalization::start() method
        // override the run method
        virtual void run();

    public:

        // Basic constructor
        AugmentedMonteCarloLocalization(ros::NodeHandle &, SampleMotionModel*, MeasurementModel*);

        // override the resample method
        virtual void resample();


};


#endif