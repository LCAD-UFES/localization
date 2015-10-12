#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

#include "Laser.hpp"
#include "Map.hpp"

class MeasurementModel {
    protected:
        // The Laser object maintains a the LaserScan message
        Laser *laser;
        // the Map object contains the OccupancyGrid
        Map *map;

    public:
        MeasurementModel(Laser *, Map *);
        // the laser must be deleted by the ParticleFilter object
        // not here
        ~MeasurementModel();
        // abstract getWeight method
        virtual void getWeight(double*) =0;
        // get the map pointer
        virtual Map* getMap() = 0;
};

#endif