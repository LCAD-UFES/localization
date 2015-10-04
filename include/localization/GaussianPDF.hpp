#ifndef GAUSSIAN_PDF_H
#define GAUSSIAN_PDF_H

#include "PDF.hpp"

class GaussianPDF : public PDF {
    public:
        // override and implements the base class run method
        virtual double sample(double);
};

#endif