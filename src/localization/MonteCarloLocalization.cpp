#include <iostream>

#include "MonteCarloLocalization.hpp"

// Constructor
MonteCarloLocalization::MonteCarloLocalization(
            ros::NodeHandle &private_nh,
            SampleMotionModel *motionModel,
            MeasurementModel *measurementModel
        ) : Xt(private_nh), motion(motionModel), measurement(measurementModel), generator(std::random_device {} ()), resample_counter(0) {

    // get the resample rate
    private_nh.param("resample_rate", resample_rate, 1);

    // get the pool size
    private_nh.param("thread_pool_size", pool_size, 5);

}

// Destructor
MonteCarloLocalization::~MonteCarloLocalization() {
    motion = nullptr;
    measurement = nullptr;
}

// inline method just to manage the acess to the SampleSet object
void MonteCarloLocalization::getSampleIndex(int &i) {

    // lock the mutex
    sample_set_mutex.lock();

    // get the actual index value
    i = sampleIndex;

    // increments
    sampleIndex++;

    // unlock the index mutex
    sample_set_mutex.unlock();

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

        // reset the SampleSet index
        sampleIndex = 0;

        // starts the new threads
        std::vector<std::thread> pool(pool_size);

        // spawn each thread
        for (int k = 0; k < pool_size; k++) {

            pool[k] = std::thread(&MonteCarloLocalization::sample, this);

        }

        // join each thread
        for (int k = 0; k < pool_size; k++) {
            pool[k].join();
        }

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

    // unlock the mutex
    mcl_mutex.unlock();
}

// this method updates the entire set
// SIMPLE SAMPLING
// moves the sample poses and then update the weight
void MonteCarloLocalization::sample() {

    // partial SampleSet weight
    double t_weight = 0.0;

    // shortcut
    // the Xt.samples pointer is updated by the resample method
    // but right now we don't need a mutex to avoid racing conditions
    // as long both methods don't run concurrently
    Sample2D *samples = Xt.samples;

    // the index
    int i;

    // get the sample index
    getSampleIndex(i);

    //
    while(i < limit) {

        // the motion model - passing sample pose by reference
        motion->samplePose2D(&samples[i].pose);

        // the measurement model - passing the Sample2D by pointer
        // the weight is assigned to the sample inside the method
        // it returns the pose weight
        t_weight  += measurement->getWeight(&samples[i]);

        // get the sample index
        getSampleIndex(i);

    }

    // lock the sample set
    sample_set_mutex.lock();

    // updates the total weight
    Xt.total_weight += t_weight;

    // unlock the sample set
    sample_set_mutex.unlock();


}

// resample the entire SampleSet - low-variance
// resampling all particles based on the weight
void MonteCarloLocalization::resample() {

    // auxiliar variables
    double M = 1.0/((double) Xt.size);
    int i = 0;
    double U;

    double ux = 0;
    double uy = 0;
    double utheta = 0;
    double x_component = 0;
    double y_component = 0;

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

        // the x coordinate mean
        ux += set[m].pose.v[0];
        // the y coordinate mean
        uy += set[m].pose.v[1];

        // the x_component
        x_component += std::cos(samples[m].pose.v[2]);

        // the y_component
        y_component += std::sin(samples[m].pose.v[2]);

    }

    // store the mean pose
    // update the coordinates and the orientation
    Xt.mean_pose.v[0] = ux*M;
    Xt.mean_pose.v[1] = uy*M;
    Xt.mean_pose.v[2] = std::atan2(y_component*M, x_component*M);


    // swap the old_set and the samples set
    Sample2D *temp = Xt.samples;
    Xt.samples = Xt.old_set;
    Xt.old_set = temp;

    // just to be sure...
    temp = nullptr;

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
void MonteCarloLocalization::getPoseArray(geometry_msgs::PoseArray &msg) {

    // shortcut
    Sample2D *samples = Xt.samples;

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
}

// return the mean pose
void MonteCarloLocalization::getMeanPose(Pose2D &p) {

    // lock the mcl
    mcl_mutex.lock();

    p.v[0] = Xt.mean_pose.v[0];
    p.v[1] = Xt.mean_pose.v[1];
    p.v[2] = Xt.mean_pose.v[2];

    // unlock the mcl
    mcl_mutex.unlock();

}


