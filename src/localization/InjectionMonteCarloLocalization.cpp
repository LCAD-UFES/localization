#include "InjectionMonteCarloLocalization.hpp"

// Basic constructor
InjectionMonteCarloLocalization::InjectionMonteCarloLocalization(
        ros::NodeHandle &private_nh,
        SampleMotionModel *motion,
        MeasurementModel *measurement
        ) : MonteCarloLocalization(private_nh, motion, measurement), sample_counter(0){

    // get the recovery alpha parameters
    private_nh.param("randon_amount_of_particles", random_amount, 0.05);
    private_nh.param("injection_rate", injection_rate, 10);

}

//
void InjectionMonteCarloLocalization::threadPeso(int inicio, int end){

    // a shortcut
    Sample2D *samples = Xt.samples;

    // temp auxiliar variable to hold the particles total weight
    double partial_weight = 0;

    //
    for(int i = inicio; i < end; i++){

        // Injection SAMPLING
        // iterate over the samples and updates everything
        // the motion model - passing sample pose by reference
        motion->samplePose2D(&samples[i].pose);

        // the measurement model - passing the Sample2D by pointer
        // the weight is assigned to the sample inside the method
        // it returns the pose weight
        partial_weight += measurement->getWeight(&samples[i]);

    }

    inject_mutex.lock();

    // updates the total_weight
    Xt.total_weight += partial_weight;

    inject_mutex.unlock();
}

// the overrided run method
void InjectionMonteCarloLocalization::run() {

    //auxiliar variables
    Sample2D *samples = Xt.samples;

    int limit = Xt.size-(Xt.size*random_amount);

    //Threads to calculate the weight of particles
    static const int num_threads = 5;
    int end;
    std::thread t[num_threads];

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if(moved){

        // reset the total weight
        Xt.total_weight = 0.0;

        if (sample_counter > injection_rate - 1) {

            // define the parts to each thread
            end = limit/num_threads;

            // random samples weight
            double random_sample_weight = 1.0/Xt.size;

            //Launch a group of threads
            for (int i = 0; i < num_threads - 1; ++i) {
                t[i] = std::thread(&InjectionMonteCarloLocalization::threadPeso, this, end*i,end*(i+1));
            }

            t[num_threads-1] = std::thread(&InjectionMonteCarloLocalization::threadPeso, this, end*(num_threads-1), limit);

            //Join the threads with the main thread
            for (int i = 0; i < num_threads; i++) {
                t[i].join();
            }

            for (int i = limit; i < Xt.size; i++) {

                // the motion model - passing sample pose by reference
                samples[i].pose = measurement->getMap()->randomPose2D();

                // the measurement model - passing the Sample2D by pointer
                // the weight is assigned to the sample inside the method
                // it returns the pose weight
                samples[i].weight = random_sample_weight;

                Xt.total_weight += random_sample_weight;

            }

            // reset the sample counter
            sample_counter = 0;

        } else {

            //Sampleamento com distrivuição aleatória
            sample_counter++;

            //define the parts to each thread
            end = Xt.size/num_threads;

            for (int i = 0; i < num_threads-1; ++i) {
                t[i] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, end*i,end*(i+1));
            }
            t[num_threads-1] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, end*(num_threads-1), Xt.size);

            //Join the threads with the main thread
            for (int i = 0; i < num_threads; i++) {
                t[i].join();
            }

        }

        // normalize
        Xt.normalizeWeights();

        // RESAMPLING
        if(resample_rate<resample_counter){

            // resampling method, it's the base class method here
            // see MonteCarloLocalization::resample()
            resample();

            resample_counter = 0;

        }else{

            resample_counter++;

        }

    }

    // unlock the mutex
    mcl_mutex.unlock();
}
