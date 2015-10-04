#include "PDF.hpp"
// just to initialize the generator object
// PDF is an abstract class
PDF::PDF() : generator(std::random_device{}() ) {}