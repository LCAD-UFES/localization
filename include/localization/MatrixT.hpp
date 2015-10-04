#ifndef MATRIX_T_H
#define MATRIX_T_H

#include <iostream>

template<typename T, int l, int c>
class MatrixT {
    public:
        T lin, col;
        T m[l][c];

        MatrixT() : lin(l), col(c) {
            clear();
        }

        // = operator overload
        void operator=(const MatrixT &inputMatrix) {
            copy(inputMatrix);
        }

        // copy the entire matrix
        MatrixT(const MatrixT &inputMatrix) : lin(l), col(c) {
            copy(inputMatrix);
        }

        // clear the entire matrix
        void clear() {
            for (int i = 0; i < lin; i++) {
                for (int j = 0; j < col; j++) {
                    m[i][j] = 0.0;
                }
            }
        }
        // copy an external matrix
        void copy(const MatrixT &inputMatrix) {
            if (lin != inputMatrix.lin || col != inputMatrix.col) {
                std::cout << std::endl << "Error! You can not copy a matrix with diferent sizes!" << std::endl;
                // clear the entire matrix
                clear();
            } else {
                // copy
                for (int i = 0; i < lin; i++) {
                    for (int j = 0; j < col; j++) {
                        m[i][j] = inputMatrix.m[i][j];
                    }
                }
            }
        }
};


#endif