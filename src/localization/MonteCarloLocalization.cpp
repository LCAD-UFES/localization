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

// the MCL algoritmh
void MonteCarloLocalization::run() {

    // auxiliar variables
    Sample2D *samples = Xt.samples;

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    motion->updateCommands(sync);

    // SIMPLE SAMPLING
    // iterate over the samples and updates everything
    for (int i = 0; i < Xt.size; i++) {

        // the motion model - passing sample pose by reference
        motion->samplePose2D(&samples[i].pose);

        // the measurement model - passing sample weight by reference
        measurement->getWeight(&samples[i]);

    }
    // RESAMPLING
    /* TODO */

    // usually the MCL returns the Xt sample set
    // what should we do here?
    // let's publish in a convenient topic
    /* TODO */

    // unlock the mutex
    mcl_mutex.unlock();
}

// spread all particles
void MonteCarloLocalization::spreadSamples(Map &map) {


    // lock the mcl
    mcl_mutex.lock();

    // spread all particle
    map.uniformSpread(&Xt);

    // unlock the mcl
    mcl_mutex.unlock();

}
