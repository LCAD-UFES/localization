#include "SampleMotionModel.hpp"

// constructor
// the generator is initialized with a random seed
SampleMotionModel::SampleMotionModel() : generator(std::random_device{}()), moved(false) {}

double SampleMotionModel::gaussianPDF(double b2) {

    // creates a normal distribution with mean == 0.0 and var as standard deviation
    std::normal_distribution<double> dist (0.0, sqrt(b2));
    return dist(generator);
}
