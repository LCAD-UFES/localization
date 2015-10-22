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

            // Injection SAMPLING
            // iterate over the samples and updates everything
            for (int i = 0; i < limit; i++) {

                // the motion model - passing sample pose by reference
                motion->samplePose2D(&samples[i].pose);

                // the measurement model - passing the Sample2D by pointer
                // the weight is assigned to the sample inside the method
                // it returns the pose weight
                Xt.total_weight  += measurement->getWeight(&samples[i]);

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
    }

    // usually the MCL returns the Xt sample set
    // what should we do here?
    // let's publish in a convenient topic

    // unlock the mutex
    mcl_mutex.unlock();
}
