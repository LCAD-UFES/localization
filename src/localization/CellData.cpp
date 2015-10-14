#include "CellData.hpp"

// basic constructor
CellData::CellData(MapCell& cs) {
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

}

// destructor
CellData::CellData() {
    // the MapCells pointer is managed by the GridMap class
    cells = nullptr;
}

// < operator overloading to use with std::priority_queue
bool CellData::operator<(CellData c) {
    // return the comparison between the occlusion distances
    return cells[MAP_INDEX(i, j)].occ_dist > c.cells[MAP_INDEX(c.i, c.j)].occ_dist; 
}