#include <iostream>
#include <cmath>
#include "GaussianPDF.hpp"

// override and implements the base class run method
double GaussianPDF::sample(double var) {

    std::normal_distribution<double> dist (0.0, sqrt(var));
    return dist(generator);

    /*
    // result normal sample
    double r = 0.0;
    // standard deviation from Variance
    double sd = sqrt(var);


    std::uniform_real_distribution<double> dist(-sd, sd);
    // see Probabilistic Robotics book
    // table 
    for (int i = 0; i < 12; i++) {
        r += dist(generator);
    }

    return r;
    */

}