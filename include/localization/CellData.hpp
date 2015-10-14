#ifndef CELL_DATA_H
#define CELL_DATA_H

#include "MapCell.hpp"

class CellData {

    public:

        // basic constructor
        CellData(MapCell*);

        // copy constructor
        CellData(const CellData&);

        // destructor
        ~CellData();

        // operator overloading, to use with std::priority_queue
        bool operator<(const CellData &) const;

        // public attributes
        // the MapCell pointer
        MapCell *cells;

        // the cell index in the unidimensional array
        unsigned int map_index;

        // the index at the grid map
        unsigned int i, j;

        // the index of the nearest obstacle
        unsigned int src_i, src_j;

};


#endif