#include "SampleMotionModel.hpp"

// constructor
// the generator is initialized with a random seed
SampleMotionModel::SampleMotionModel() : generator(std::random_device{}()) {}

SampleMotionModel::gaussianPDF(double var) {
    // creates a normal distribution with mean == 0.0 and sqrt(var) as standard deviation
    std::normal_distribution<double> dist (0.0, sqrt(var));
    // return
    return dist(generator);
}
