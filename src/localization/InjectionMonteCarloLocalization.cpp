#include "InjectionMonteCarloLocalization.hpp"

// Basic constructor
InjectionMonteCarloLocalization::InjectionMonteCarloLocalization(
        ros::NodeHandle &private_nh,
        SampleMotionModel *motion,
        MeasurementModel *measurement
        ) : MonteCarloLocalization(private_nh, motion, measurement), sample_counter(0){

    // get the recovery alpha parameters
    private_nh.param("number_random_particles", number_injections, 0.05);
    private_nh.param("injection_times", injection_times, 10);

}

void InjectionMonteCarloLocalization::threadPeso(int inicio, int fim){

    Sample2D *samples = Xt.samples;
    double t_weight=0;

    //#pragma omp parallel for default(none) private(i, total_weight) shared(samples)
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

        t_weight = measurement->getWeight(&samples[i]);

        //#pragma omp critical
        {
            inject_mutex.lock();
            Xt.total_weight  += t_weight;
            inject_mutex.unlock();
        }
    }

}

// the overrided run method
void InjectionMonteCarloLocalization::run() {

    //auxiliar variables
    Sample2D *samples = Xt.samples;
    int limit = Xt.size-(Xt.size*number_injections);
    //Threads to calculate the weight of particles
    static const int num_threads = 5;
    int fim;
    std::thread t[num_threads];

    // update the LaserScan and the GridMap if necessary and returns the laser TimeStamp
    sync = measurement->update();

    // get the available commands
    bool moved = motion->update(sync);

    if(moved){
        // reset the total weight
        Xt.total_weight = 0.0;

        if (sample_counter > injection_times - 1) {
            //define the parts to each thread
            fim = limit/num_threads;
            //peso para as particulas aleatorias
            double pesoInject = 0.00000000005;

            //Launch a group of threads
            for (int i = 0; i < num_threads-1; ++i) {
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
                samples[i].weight = pesoInject;
                Xt.total_weight  += pesoInject;

            }
            sample_counter=0;

        }
        else{

            //Sampleamento com distrivuição aleatória
            sample_counter++;

            //define the parts to each thread
            fim = Xt.size/num_threads;

            for (int i = 0; i < num_threads-1; ++i) {
                t[i] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, fim*i,fim*(i+1));
            }
            t[num_threads-1] = std::thread(&InjectionMonteCarloLocalization::threadPeso,this, fim*(num_threads-1), Xt.size);

            //Join the threads with the main thread
            for (int i = 0; i < num_threads; i++) {
                t[i].join();
            }

        }

        // normalize
        Xt.normalizeWeights();

        // RESAMPLING
        if(resample_rate<resample_counter){
            resample();
            resample_counter = 0;
        }else{
            resample_counter++;
        }
    }

    // unlock the mutex
    mcl_mutex.unlock();
}
