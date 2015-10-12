#include "SampleSet.hpp"
#include <cmath>

// basic constructor
SampleSet::SampleSet(const ros::NodeHandle &private_nh) {
    // get the size parameter
    // if not found, it'll be 800'
    private_nh.param( (std::string) "sample_set_size", size, 800);

    // get the min and max samples
    private_nh.param("min_sample_set_size", min, 100);
    private_nh.param( (std::string) "max_sample_set_size", max, 10000);

    // allocate the array of 2D samples
    newSamples();
}

// destructor
SampleSet::~SampleSet() {
    delete samples;
}

// allocate the samples in memmory
void SampleSet::newSamples() {
    // the sample set size limits
    if (min <= size && size <= max) {
        samples = new Sample2D[size]();
        if (nullptr == samples) {
            throw std::bad_alloc();
        }
        // reset the samples
        resetSamples();
    }
}

// reset all poses to zero and the weigths to 1/size
void SampleSet::resetSamples() {
    if (nullptr != samples) {
        // size must be greater than zero
        double min_w = 1/size;

        for (int i = 0; i < size; i++) {
            samples[i].pose = { 0.0, 0.0, 0.0 };
            samples[i].weight = min_w;
        }
    }
}

// uniform random distribution
void SampleSet::uniformSpread(Map &map) {

    // verify if the map is available
    if (map.mapReceived()) {

        // spread
        std::cout << "Spreading all particles!" << std::endl;

        // get the map width
        int width = map.geWidth();
        // get the map height
        int height = map.getHeight();
        // get the map scale
        double scale = map.getScale();
        // get the map origin_x
        double origin_x = map.getOriginX();
        // get the map origin_y
        double origin_y = map.getOriginY();

        // get the available cells index
        std::vector<int> indexes = map.getAvailableCellsIndex();

        // set a generator engine
        std::default_random_engine generator(std::random_device {} ());

        // set a uniform distribution
        std::uniform_int_distribution<int> uniform_dist(0.0, indexes.size() - 1);

        // set another uniform distribution
        std::uniform_real_distribution<double> angle_dist(0.0, std::atan(1.0)*8);

        // set a normal distribution
        std::normal_distribution<double> normal_dist(0.0, 0.05);

        // tmp variables
        int index;
        double *pose;
        int g_x, g_y, w_x, w_y;

        // sledgehammer programing style? =-/
        // create random poses based on the unnoccupied cells
        for (int i = 0; i < size; i++) {

            // a simple pointer to avoid repetitive writing
            pose = samples->pose.v;

            // get a random index from the uniform_int_distribution
            index = uniform_dist(generator);

            // get the grid x coord
            g_x = indexes[index]/width;
            // get the grid y coord
            g_y = indexes[index] % width;

            // now we have the grid index and a we can 
            // convert to world coords and assign to the pose value
            // with a simple noise
            pose[0] = (origin_x + (g_x - width/2)*scale) + normal_dist(generator);
            pose[1] = (origin_y + (g_y - height/2)*scale) + normal_dist(generator);
            // random orientation
            pose[2] = angle_dist(generator);
        }
    }
}