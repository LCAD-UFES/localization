#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include <string>

class MCL {
    private:
        // The set of samples
        SampleSet Xt;

    public:
        // it needs to specify the velocity and measurement models
        // Velocity Models: "vel" == "velocity" or "odom" == odometry
        // Measurement Models: "beam" == beam range find or "likelyhood" == likelyhood field
        MCL();
};

#endif