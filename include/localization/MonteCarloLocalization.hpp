#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include <string>
#include "SampleMotionModel.hpp"
#include "MeasurementModel.hpp"
#include "MapServer.hpp"

class MCL {
    private:
        // The set of samples
        SampleSet Xt;

    public:
        // this constructor receives the amount of samples
        // it needs to specify the velocity and measurement models
        // Velocity Models: "vel" == "velocity" or "odom" == odometry
        // Measurement Models: "beam" == beam range find or "likelyhood" == likelyhood field
        // and finally the map filename as the last argument
        MCL(unsigned int, std::string, std::string, std::string);
        // or you can pass the the Models and the MapServer as well
        MCL(unsigned int, SampleMotionModel*, MeasurementModel*, MapServer*);

        // the run method
        void run();
};

#endif