#include "Sample2D.hpp"

// constructor
Sample2D::Sample2D() : pose(), weight(0.0) {}

// constructor
Sample2D::Sample2D(double w) : pose(), weight(w) {}

// copy constructor
Sample2D::Sample2D(const Sample2D &s) : pose(s.pose), weight(s.weight) {}