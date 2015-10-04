#ifndef PDF_H
#define PDF_H

#include <iostream>
#include <random>

class PDF {
    protected:
        // generator
        std::default_random_engine generator;
    public:
        // constructor
        PDF();
        // abstract method
        virtual double sample(double) =0;
};

#endif