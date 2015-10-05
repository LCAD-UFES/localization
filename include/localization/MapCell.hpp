#ifndef MAP_CELL_H
#define MAP_CELL_H

struct MapCell {
    // occupancy state (-1 == free, 0 == unknow, +1 == occ)
    int occ_state;
    // distance to the nearest ocupied cell
    double occ_dist;
};

#endif