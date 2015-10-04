#include "UniformPDF.hpp"
#include <cmath>

double UniformPDF::sample(double var) {

    double dp = sqrt(var);
    // uniform distribution

    std::uniform_real_distribution<double> distribution(-dp, dp);

    return distribution(generator);
}