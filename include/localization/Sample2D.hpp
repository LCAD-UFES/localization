#ifndef SAMPLE_2D_H
#define SAMPLE_2D_H

#include "Pose2D.hpp"

class Sample2D {

    public:
        // base constructor
        Sample2D();

        // simple constructor
        Sample2D(double);

        // copy constructor
        Sample2D(const Sample2D&);

        /* Atributes */
        // 2D Pose
        Pose2D pose;

        // the particle weight
        double weight;

};

#endif