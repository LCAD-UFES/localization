#ifndef CELL_DATA_H
#define CELL_DATA_H

#include "MapCell.h"

class CellData {
    public:

        // basic constructor
        CellData(MapCell&);

        // copy constructor
        CellData(const CellData&);

        // destructor
        ~CellData();

        // operator overloading, to use with std::priority_queue
        bool operator<(CellData);

        // public attributes
        // the MapCell pointer
        MapCell *cells;

        unsigned int i, j;
        unsigned int src_i, src_j;

};


#endif