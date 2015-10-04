#ifndef PDF_H
#define PDF_H

#include <iostream>
#include <random>

class PDF {
    protected:
        // generator
        // it needs the c++11 standard... a bad choice?
        // if We can't use c++11 with ros then We'll have to find another solution
        std::default_random_engine generator;
    public:
        // constructor
        PDF();
        // abstract method
        virtual double sample(double) =0;
};

#endif