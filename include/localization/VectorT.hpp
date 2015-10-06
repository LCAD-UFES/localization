#ifndef VECTOR_T_H
#define VECTOR_T_H

#include <iostream>

template<typename T, int s>
class VectorT {
    public:
        int size;
        T v[s];
        // simple constructor
        VectorT() : size(s) {
            // clear the entire vector
            clear();
        }

        // = operator overload
        void operator=(VectorT inputVector) {
            copy(inputVector);
        }

        // copy constructor
        VectorT(const VectorT &inputVector) : size(s) {
            copy(inputVector);
        }

        // clear the entire vector
        void clear() {
            for (int i = 0; i < size; i++) {
                v[i] = 0.0;
            }
        }

        // copy
        void copy(const VectorT &inputVector) {
            if (size != inputVector.size) {
                std::cout << std::endl << "Error! You can not copy a vector with diferent sizes!" << std::endl;
                clear();
            } else {
                for (int i = 0; i < size; i++) {
                    v[i] = inputVector.v[i];
                }
            }
        }

};

// simple definition to be used in double v[3] cases
typedef VectorT<double, 3> VectorT_DBL_3;

// simple definition to be used in double v[6] cases
typedef VectorT<double, 6> VectorT_DBL_6;

#endif