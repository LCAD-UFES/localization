#ifndef MAP_CELL_H
#define MAP_CELL_H

struct MapCell {

    // occupancy state (-1 = free, 0 = unknow, +1 = occ)
    int occ_state;

    // Distance to the nearest occupied cell
    double occ_dist;

};

#endif