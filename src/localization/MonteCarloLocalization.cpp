#include <iostream>

#include "MonteCarloLocalization.hpp"

// Constructor
MonteCarloLocalization::MonteCarloLocalization(
            ros::NodeHandle &private_nh,
            SampleMotionModel *motionModel,
            MeasurementModel *measurementModel
        ) : Xt(private_nh), motion(motionModel), measurement(measurementModel), generator(std::random_device {} ()), resample_counter(0) {

    // private n
    private_nh.param("resample_rate", resample_rate, 30);

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

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if(moved) {

        // reset the total weight
        Xt.total_weight = 0.0;

        // shortcut
        Sample2D *samples = Xt.samples;

        // SIMPLE SAMPLING
        // iterate over the samples and updates everything
        for (int i = 0; i < Xt.size; i++) {

            // the motion model - passing sample pose by reference
            motion->samplePose2D(&samples[i].pose);

            // the measurement model - passing the Sample2D by pointer
            // the weight is assigned to the sample inside the method
            // it returns the pose weight
            Xt.total_weight  += measurement->getWeight(&samples[i]);

        }

        // normalize
        // normalize
        Xt.normalizeWeights();

        // RESAMPLING
        if (resample_rate < resample_counter) {

            // resampling
            resample();

            // reset the resample_counter
            resample_counter = 0;

        } else {

            // increments the resample_counter
            resample_counter++;
        }
    }

    // usually the MCL returns the Xt sample set
    // what should we do here?
    // let's publish in a convenient topic

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

// return a copy of the Sample2D
geometry_msgs::PoseArray MonteCarloLocalization::getPoseArray() {

    // shortcut
    Sample2D *samples = Xt.samples;

    // build a copy of the Sample2D array
    geometry_msgs::PoseArray msg;

    // set the frame_id
    msg.header.frame_id = "map";

    // lock the mcl
    mcl_mutex.lock();

    // get the current timestamp
    msg.header.stamp = ros::Time::now();

    // resize the pose array
    msg.poses.resize(Xt.size);

    // copy the poses
    for (int i = 0; i < Xt.size; i++) {
        // copy the current pose transformed by the appropriate tf
        // Euler to Quaternion
        tf::poseTFToMsg(
                            tf::Pose(
                                tf::createQuaternionFromYaw(samples[i].pose.v[2]),
                                tf::Vector3(samples[i].pose.v[0], samples[i].pose.v[1], 0)),
                            msg.poses[i]
                       );
    }

    // unlock the mcl
    mcl_mutex.unlock();

    samples = nullptr;

    return msg;

}

// resample the entire SampleSet - low-variance
// resampling all particles based on the weight
void MonteCarloLocalization::resample() {

    // auxiliar variables
    double M = 1.0/((double) Xt.size);
    int i = 0;
    double U;

    // shortcuts
    Sample2D *samples = Xt.samples;
    Sample2D *set = Xt.old_set;

    // a uniform distribution
    // from zero to size - 1, our SampleSet size
    std::uniform_real_distribution<double> uniform(0, M);

    // get a random value
    double r = uniform(generator);

    // get the first weight
    double c = samples[0].weight;

    // reset total weight
    Xt.total_weight = 0.0;

    // iterate over the entire SampleSet
    for (int m = 0; m < Xt.size; m++) {

        U = r + (m)*M;

        while (U > c) {

            i = i + 1;
            c += samples[i].weight;

        }

        // copy the x coordinate
        set[m].pose.v[0] = samples[i].pose.v[0];

        // copy the y coordinate
        set[m].pose.v[1] = samples[i].pose.v[1];

        // copy the yaw orientation
        set[m].pose.v[2] = samples[i].pose.v[2];

        // copy the sample weight
        set[m].weight = samples[i].weight;

        // updates the new total_weight
        Xt.total_weight += samples[i].weight;

    }

    // swap the old_set and the samples set
    Sample2D *temp = Xt.samples;
    Xt.samples = Xt.old_set;
    Xt.old_set = temp;

    // just to be sure...
    temp = nullptr;

    Xt.normalizeWeights();

}
