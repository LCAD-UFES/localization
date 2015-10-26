#include "InjectionMonteCarloLocalization.hpp"

// Basic constructor
InjectionMonteCarloLocalization::InjectionMonteCarloLocalization(
        ros::NodeHandle &private_nh,
        SampleMotionModel *motion,
        MeasurementModel *measurement
        ) : MonteCarloLocalization(private_nh, motion, measurement), sample_counter(0){

    // get the recovery alpha parameters
    private_nh.param("number_random_particles", number_injections, 0.20);
    private_nh.param("injection_times", injection_times, 10);

}

void InjectionMonteCarloLocalization::threadPeso(int inicio, int fim){
    Sample2D *samples = Xt.samples;
    //SampleMotionModel moti = motion;
    //MeasurementModel meas = measurement;
    double total_weight=0;

    for(int i=inicio; i<fim; i++){
        // Injection SAMPLING
        // iterate over the samples and updates everything
            // the motion model - passing sample pose by reference
        //inject_mutex.lock();
            motion->samplePose2D(&samples[i].pose);
        //inject_mutex.unlock();

            // the measurement model - passing the Sample2D by pointer
            // the weight is assigned to the sample inside the method
            // it returns the pose weight

        total_weight = measurement->getWeight(&samples[i]);

        inject_mutex.lock();
        Xt.total_weight  += total_weight;
        inject_mutex.unlock();
    }
     //mcl_mutex.unlock();
    //return total_weight;
}

// the overrided run method
void InjectionMonteCarloLocalization::run() {

    //auxiliar variables
    Sample2D *samples = Xt.samples;
    int size = Xt.size;
    int limit = size-(size*number_injections);

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if(moved){
        // reset the total weight
        Xt.total_weight = 0.0;

        if (sample_counter > injection_times - 1) {

            //Threads to calculate the weight of particles
            static const int num_threads = 5;
            int fim = limit/num_threads;
            std::thread t[num_threads];
//            //the first
            t[0] = std::thread(&InjectionMonteCarloLocalization::threadPeso, this, 0, fim);
            //Launch a group of threads
            for (int i = 1; i < num_threads-1; ++i) {
                t[i] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, fim*i,fim*(i+1));
            }
            t[num_threads-1] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, fim*(num_threads-1), limit);

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
                samples[i].weight = 1.0;
                Xt.total_weight  += 1.0;

            }
            sample_counter=0;

        }
        else{

            sample_counter++;

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

        }

        // normalize
        // normalize
        Xt.normalizeWeights();

        // RESAMPLING
        if(resample_rate<resample_counter){
            resample();
            resample_counter = 0;
        }
        else{
            resample_counter++;
        }
    }

    // usually the MCL returns the Xt sample set
    // what should we do here?
    // let's publish in a convenient topic

    // unlock the mutex
    mcl_mutex.unlock();
}
