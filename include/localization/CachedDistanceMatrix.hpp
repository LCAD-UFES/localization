#ifndef CACHED_DISTANCE_MATRIX_H
#define CACHED_DISTANCE_MATRIX_H

class CachedDistanceMatrix {
    public:
        // constructor
        CachedDistanceMatrix(double, double);
        ~CachedDistanceMatrix();

        // public attributes
        double **distances;
        double scale;
        double max_distance;
        double cell_radius;
};


#endif