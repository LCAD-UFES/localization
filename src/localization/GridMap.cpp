#include "GridMap.hpp"

// basic constructor
// max_occ_dist default to 2.0
GridMap::GridMap() : scale(1), max_occ_dist(0.5), cells(nullptr), stamp(0) {}

// Copy Constructor
GridMap::GridMap(const GridMap &g) : 
                                    origin_x(g.origin_x),
                                    origin_y(g.origin_y),
                                    scale(g.scale),
                                    width(g.width),
                                    height(g.height),
                                    max_occ_dist(g.max_occ_dist) {

    size = g.width*g.height;

    // allocate the map cells
    cells = new MapCell[size];
    if (nullptr == cells) {
        throw std::bad_alloc();
    }

    // copy all cells
    for (int i = 0; i < size; i++) {
        cells[i] = g.cells[i];
    }
}

// destructor
GridMap::~GridMap() {
    if (nullptr != cells) {
        delete cells;
    }
}

// update the grid mad
void GridMap::updateGridMap(const nav_msgs::OccupancyGrid &map_msg) {

    // get the size
    int grid_size = map_msg.info.width*map_msg.info.height;


    // maybe we don't need to delete de cells
    if (width != map_msg.info.width || height != map_msg.info.height)  {

        if (nullptr != cells) {
            delete cells;
        }

        // update the map infos
        width = map_msg.info.width;
        height = map_msg.info.height;
        size = grid_size;

        // realocate the cells
        cells = new MapCell[grid_size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }
    }

    scale  =  map_msg.info.resolution;
    origin_x = map_msg.info.origin.position.x + (map_msg.info.width/2)*map_msg.info.resolution;
    origin_y = map_msg.info.origin.position.y + (map_msg.info.height/2)*map_msg.info.resolution;
    orientation = map_msg.info.origin.orientation;

    // copy the occupancy state
    for (int i = 0; i < grid_size; i++) {
        if (0.65 < map_msg.data[i]) {
            cells[i].occ_state = +1;
        } else if (map_msg.data[i] >= 0) {
            cells[i].occ_state = -1;
        } else {
            cells[i].occ_state = 0;
        }
    }

    // finally, the timestamp
    stamp = map_msg.header.stamp;

}

// update the grid mad, now with another GridMap, overloading
void GridMap::copy(const GridMap &g) {

    // get the size
    int grid_size = g.width*g.height;

    // maybe we don't need to delete de cells
    if (width != g.width || height != g.height)  {

        if (nullptr != cells) {
            delete cells;
        }

        // update the map infos
        width = g.width;
        height = g.height;
        size = grid_size;

        // realocate the cells
        cells = new MapCell[grid_size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }

    } else if (nullptr == cells) {

        // realocate the cells
        cells = new MapCell[grid_size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }

    }

    scale  =  g.scale;
    origin_x = g.origin_x;
    origin_y = g.origin_y;
    orientation = g.orientation;


    // copy the occupancy state
    for (int i = 0; i < grid_size; i++) {
        cells[i] = g.cells[i];
    }

    // the max occlusion distance parameter
    max_occ_dist = g.max_occ_dist;

    // finally, the timestamp
    stamp = g.stamp;

}

// get the pre-computed likelihood
// i = x and j = y in GridMap coordinates
double GridMap::getMinDistance(int i, int j) {

    // get the correct index in our unidimensional array
    int index = MAP_INDEX(i, j);

    // verifing the bounds
    if (index >= 0 && index < width*height) {

        return cells[index].occ_dist;

    }

    // otherwise returns the mas occlusion distance
    return max_occ_dist;

}

void GridMap::enqueue(
    unsigned int i,
    unsigned int j,
    unsigned int src_i,
    unsigned int src_j, 
    std::priority_queue<CellData> &Q,
    double **distances,
    int cell_radius,
    unsigned char *marked
    ) {

    // if the current cell is already marked we can leave it with the same value
    // it means that this cell was found first by another obstace, so it is closer to another obstacle
    if (marked[MAP_INDEX(i, j)]) {
        return;
    }

    // get the displacement
    unsigned int di = abs(i - src_i);
    unsigned int dj = abs(j - src_j);

    // get the pre-computed distance
    double dist = distances[di][dj];

    // verify the bound limit given by the cell_radius
    if(dist > cell_radius) {
        return;
    }

    // set the appropriate distance of the given cell inside our MapCell grid
    cells[MAP_INDEX(i, j)].occ_dist = dist*scale;

    // create a new cell and save the current into the Priority Queue
    // so we can find its neighbors and get the distance from the (src_i, src_j)
    // if the distance of any neighbor exceeds the cell_radius limit or already marked
    // the function will return in the previous conditions tests
    CellData c(cells);

    // save the actual cell index
    c.i = i;
    c.j = j;

    // save the actual unidimensional index
    c.map_index = MAP_INDEX(i, j);

    // save the first source index (src_i, src_j)
    // so the neighbors will be compared to the same source
    c.src_i = src_i;
    c.src_j = src_j;

    // save this cells to the Priority Queue
    // the priority is given by the occ_dist attribute, see the operator overloading
    Q.push(c);

    // change the marked flag
    // we can be sure about this situation. The Priority Queue
    // ensures that this cell (i, j) is found first by the nearest obstacle cell
    marked[MAP_INDEX(i, j)] = 1;

}
 
// pre-computing the nearest neighbor
void GridMap::nearestNeighbor() {

    // get the occlusion radius
    int cell_radius = max_occ_dist/scale;

    // build a cached distance matrix
    double **distances = new double*[cell_radius + 2];

    // fill the distances
    for (int i = 0; i <= cell_radius + 1; i++) {
        distances[i] = new double[cell_radius + 2];
        for (int j = 0; j <= cell_radius + 1; j++) {
            // the euclidian distances
            distances[i][j] = sqrt(i*i + j*j);
        }
    }

    // just to flag the marked ones
    unsigned char *marked = new unsigned char[size];
    for (int i = 0; i < size/10000; i++) {
        marked[i] = 0;
    }

    // the priority queue of CellData
    // this CellData class has the < operator overloaded
    std::priority_queue<CellData> Q;

    // build an auxiliar CellData
    // its passed by reference
    CellData c(cells);

    // get all obsctacle cells
    for (int i = 0; i < width; i++) {

        // copy the i index to the CellData
        c.i = c.src_i = i;

        for (int j = 0; j < height; j++) {

            // verify if the current cell is occupied
            if(1 == cells[MAP_INDEX(i, j)].occ_state) {

                // update the occlusion to zero
                cells[MAP_INDEX(i, j)].occ_dist = 0.0;

                //copy the j index to the CellData
                c.j = c.src_j = j;

                // update the unidimensional index
                c.map_index = MAP_INDEX(i,j);

                // mark
                marked[MAP_INDEX(i, j)] = 1;

                // keep the obstacle cell
                Q.push(c);

            } else {

                // assignt the max occlusion distance
                cells[MAP_INDEX(i, j)].occ_dist = max_occ_dist;

            }
        }
    }

    // now we have all obstacle cells
    // let's start!
    while(!Q.empty()) {

        // get the hight priority CellData
        // based on the max_occ_dist of that cell in the grid
        // se the CellData class and the < operator overloading
        CellData current(Q.top());

        // verify the cases
        // maybe the current cell is next to the matrix borders
        // in those cases, there aren't neighbors at a given side

        // left?
        if(current.i > 0) {
            // there's a neighbor a the left side
            enqueue(current.i - 1, current.j, current.src_i, current.src_j, Q, distances, cell_radius, marked);
        }

        // above?
        if (current.j > 0) {
            // there's a neighbor above te current cell
            enqueue(current.i, current.j - 1, current.src_i, current.src_j, Q, distances, cell_radius,  marked);
        }

        // right?
        if (current.i < width - 1) {
            // there's a neighbor at the right side
            enqueue(current.i + 1, current.j, current.src_i, current.src_j, Q, distances, cell_radius, marked);
        }

        // below?
        if (current.j < height - 1) {
            enqueue(current.i, current.j + 1, current.src_i, current.src_j, Q, distances, cell_radius,  marked);
            // there's a neighbor below the current cell
        }

        // remove the element
        Q.pop();
    }

    // free memmory
    // remove the pre-computed distances
    delete [] distances;

    // remove the marked vector
    delete marked;

}