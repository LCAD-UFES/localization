#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

#include <mutex>

#include "Map.hpp"
#include "Laser.hpp"
#include "Sample2D.hpp"

class MeasurementModel {
    protected:

        // a pointer to the global Laser
        Laser *laser;

        // our internal LaserScan representation
        sensor_msgs::LaserScan ls_scan;

        // the laser max beams
        int max_beams;

        // the step
        int step;

        // the Map object is a container to our GridMap
        Map *map;

        // Our internal GridMap representation
        GridMap grid;

    public:

        // basic constructor
        MeasurementModel(const ros::NodeHandle &, Laser*, Map *);

        // the laser must be deleted by the ParticleFilter object
        // not here
        ~MeasurementModel();

        // abstract getWeight method
        virtual double getWeight(Sample2D*) =0;

        // update the LaserScan and if necessary the GridMap also
        virtual ros::Time update() =0;

        // get the map pointer
        virtual Map* getMap();
};

#endif
