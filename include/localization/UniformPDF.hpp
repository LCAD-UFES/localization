#ifndef UNIFORM_PDF_H
#define UNIFORM_PDF_H

#include <random>
#include "PDF.hpp"

class UniformPDF : public PDF {
    public:
        // override and implements the base class run method
        virtual double sample(double);
};

#endif