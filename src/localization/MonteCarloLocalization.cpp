#include <iostream>

#include "MonteCarloLocalization.hpp"
// Constructor
MonteCarloLocalization::MonteCarloLocalization(
            ros::NodeHandle &private_nh,
            SampleMotionModel *motionModel,
            MeasurementModel *measurementModel
        ) : Xt(private_nh), motion(motionModel), measurement(measurementModel), mcl_thread() {

    // 
    if (mcl_thread.joinable())
        mcl_thread.join();
}

// Destructor
MonteCarloLocalization::~MonteCarloLocalization() {
    motion = nullptr;
    measurement = nullptr;
}

//
void MonteCarloLocalization::run() {

    std::cout << std::endl << "Unlocking!";
    // unlock the mutex
    mcl_mutex.unlock();
}


// start a thread
// it starts a thread that executes the run() method and exits smoothly
void MonteCarloLocalization::start() {

    // look at the mutex
    // it is unlocked when the run() method is finished
    mcl_mutex.lock();

    // spawns a new thread
    t = std::thread(&MonteCarloLocalization::run, this);

    // detach from the current thread
    // we can proceed and leave this thread on it's own
    // avoiding the blocking thread::join()
    mcl_thread.detach();
}
