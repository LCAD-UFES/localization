#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

class MeasurementModel {
    // it needs to assign to the correct topics
    //
    public:
        // abstract method
        virtual void sample(CommandReader *)
};

#endif