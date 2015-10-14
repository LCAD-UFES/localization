#include "CellData.hpp"

// basic constructor
CellData::CellData(MapCell* cs) {
    cells = cs;
}

// the copy constructor
CellData::CellData(const CellData& cd) {

    // use the same MapCell address
    cells = cd.cells;

    // get the indexes
    i = cd.i;
    j = cd.j;
    src_i = cd.src_i;
    src_j = cd.src_j;
    // get the unidimensional index
    map_index = cd.map_index;   

}

// destructor
CellData::~CellData() {
    // the MapCells pointer is managed by the GridMap class
    cells = nullptr;
}

// < operator overloading to use with std::priority_queue
bool CellData::operator<(const CellData &c) const {
    // return the comparison between the occlusion distances
    return cells[map_index].occ_dist > c.cells[map_index].occ_dist; 
}