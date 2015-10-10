#include <iostream>

#include "MonteCarloLocalization.hpp"
// Constructor
MonteCarloLocalization::MonteCarloLocalization(
            ros::NodeHandle &private_nh,
            SampleMotionModel *motionModel,
            MeasurementModel *measurementModel
        ) : Xt(private_nh), motion(motionModel), measurement(measurementModel) {
}

// Destructor
MonteCarloLocalization::~MonteCarloLocalization() {
    motion = nullptr;
    measurement = nullptr;
}

// spawns a new thread
// it starts a thread that executes the run() method and exits smoothly
void MonteCarloLocalization::start() {

    // look at the mutex
    if (mcl_mutex.try_lock()) {
        // it is unlocked when the run() method is finished
        // spawns a new thread
        std::thread mcl_thread(&MonteCarloLocalization::run, this);

        // detach from the current thread
        // we can proceed and leave this thread on it's own
        // avoiding the blocking thread::join()
        mcl_thread.detach();
    }
}


//
void MonteCarloLocalization::run() {

    // get the last laser
    // Sample the Xt
    Xt.sample(motion, measurement);

    // resample??
    // Xt.resample();

    // usually the MCL returns the Xt sample set
    // what should we do here?
    /* TODO */

    
    // unlock the mutex
    mcl_mutex.unlock();
}