#include "BeamRangeFinderModel.hpp"
#include <cmath>
#include <math.h>
#include <time.h>


// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(private_nh, ls, m) {

    // get the z_hit parameter
    private_nh.param("beam_z_hit", z_hit, 0.7);

    // get the z_short parameter
    private_nh.param("beam_z_short", z_short, 0.2);

    // get the z_max parameter
    private_nh.param("beam_z_max", z_max, 0.05);

    // get the z_rand parameter
    private_nh.param("beam_z_rand", z_rand, 0.05);

    // get the sigma_hit parameter
    private_nh.param("beam_sigma_hit", sigma_hit, 0.2);

    // get the lambda short
    private_nh.param("beam_lambda_short", lambda_short, 0.2);

    // get the max_beams parameter, see MeasurementModel base class
    private_nh.param("laser_max_beams", max_beams, 60);

    // pre-computing a few things
    if (0.0 == sigma_hit) {
        sigma_hit = 0.2;
    }

    // update the sigma_den
    sigma_den = -0.5/(sigma_hit*sigma_hit);

}

// computing the probability of a given pose
double BeamRangeFinderModel::getWeight(Sample2D *sample) {

        // if the current pose is inside a obstacle...
    if (!grid.validPose(sample->pose.v[0], sample->pose.v[1])) {

        sample->weight *= 0.001;

        return sample->weight;

    }

    // probability result
    double p =  1.0;

    // the observation beraing and range and the computed ray
    double obs_bearing, obs_range, ray_range;

    // shortcuts
    double px = sample->pose.v[0];
    double py = sample->pose.v[1];
    double theta = sample->pose.v[2];

    // the start point - the robot's pose
    int x0 = std::floor((px - grid.origin_x)*inverse + 0.5) + grid_width_2;
    int y0 = std::floor((py - grid.origin_y)*inverse + 0.5) + grid_height_2;

    // the width and height
    double width_2_scale = grid.resolution*grid_width_2;
    double height_2_scale = grid.resolution*grid_height_2;

    // shortcuts
    double px_displacement = (px + width_2_scale);
    double py_displacement = (py + height_2_scale);

    // the endpoints
    int x1, y1, x2, y2;

    // the cell displacements
    int dx, dy;

    // the laser beam direction
    int ix, iy, fc_x, fc_y;

    // the range max
    float range_max = ls_scan.range_max + 1;

    // aux variables,
    float ngl_x, ngl_y, Tx, Ty, tx, ty, end_x, end_y, px_, py_, fabs_dx_, fabs_dy_;

    bool found;

    // iterate the beams and calculates the ray casting
    // and then computes the probability of the current pose
    for (int i = 0; i < ls_scan.size; i += step) {

        // copy the range and the angle
        obs_range = ls_scan.ranges[i][0];
        obs_bearing = ls_scan.ranges[i][1];

        // reset the x1 and y1
        x1 = x0;
        y1 = y0;

        // the endpoint - where the laser beam hits in the map
        x2 = std::floor((px + range_max*std::cos(theta + obs_bearing) - grid.origin_x)*inverse + 0.5) + grid_width_2;
        y2 = std::floor((py + range_max*std::sin(theta + obs_bearing) - grid.origin_y)*inverse + 0.5) + grid_height_2;

        // verify the bounds
        if (0 > x2) {

            x2 = 0;

        } else if (x2 >= grid.width) {

            x2 = grid.width - 1;

        }

        if (0 > y2) {

            y2 = 0;

        } else if (y2 >= grid.height) {

            y2 = grid.height -1;

        }

        // computes the displacement
        dx = (x2 - x0);

        // get the direction and the floor or ceil flag
        if (dx > 0) {
            ix = fc_x = 1;
        } else {
            ix = -1;
            fc_x = 0;
        }
        // get the inverse absolute x displacement
        fabs_dx_ = 1.0/std::fabs(dx);

        // computes the displacement
        dy = (y2 - y0);

        // get the direction and the floor or ceil flag
        if (dy > 0) {
            iy = fc_y = 1;
        } else {
            iy = -1;
            fc_y = 0;
        }

        // get the inverse absolute y displacement
        fabs_dy_ = 1.0/std::fabs(dy);;

        // the distance to nearest grid line
        ngl_x = std::fabs((x0 + fc_x)*grid.resolution - px_displacement);
        ngl_y = std::fabs((y0 + fc_y)*grid.resolution - py_displacement);

        // computes the max time to cross any cell
        // the cell may have different sizes in a general case
        Tx = (dx == 0) ? std::numeric_limits<float>::infinity() : grid.resolution*fabs_dx_;
        Ty = (dy == 0) ? std::numeric_limits<float>::infinity() : grid.resolution*fabs_dy_;

        // computes the time to arrive at the last evaluated cell
        tx = (dx == 0) ? std::numeric_limits<float>::infinity() : ngl_x*fabs_dx_;
        ty = (dy == 0) ? std::numeric_limits<float>::infinity() : ngl_y*fabs_dy_;

        // set the flag to false
        found = false;

        // see Bresenham algorithm
        while (x1 != x2 && y1 != y2) {

            if (tx < ty) {

                x1 += ix;
                ty -= tx;
                tx = Tx;

            } else {

                y1 += iy;
                tx -= ty;
                ty = Ty;

            }

            // verify the current cell, maybe we have an obstacle
            if (1 == grid.cells[x1 + y1*grid.width].occ_state) {

                end_x = (dx == 0) ? ngl_x : tx * fabs(dx);
                end_y = (dy == 0) ? ngl_y : ty * fabs(dy);

                px_ = (x1 + fc_x)*grid.resolution - (end_x*ix + width_2_scale);
                py_ = (y1 + fc_y)*grid.resolution - (end_y*iy + height_2_scale);

                // where the
                px_ += grid.origin_x    ;
                py_ += grid.origin_y;

                // computes the sim ray range
                ray_range = std::sqrt((px_ - px)*(px_ - px) + (py_ - py)*(py_ - py));

                // updates the probability
                p += prob(obs_range, ray_range);

                // set the flag
                found = true;

                // break the while loop
                break;

            }

        }

        // the range max case
        if (!found) {
            // max range
            p += prob(obs_range, range_max);

        }

    }

    // just to be shure
    assert(p >= 0);

    // save the weight
    sample->weight = p;

    // returns the weigth, the MonteCarlo object will add this value to the SampleSet total_weight
    return sample->weight;

}

// update the Scan and the GridMap objects and return a timestamp to sync the commands
ros::Time BeamRangeFinderModel::update() {

    // update the laser
    laser->getScan(&ls_scan);

    // update the step
    step = ls_scan.size/(max_beams -1);
    // being cautious
    if (1 > step) {
        step = 1;
    }

    // get the p_rand
    p_rand = 1.0/ls_scan.range_max;


    // update the GridMap if necessary
    map->getGridMap(&grid);

    // get the widht/2
    grid_width_2 = grid.width >> 1;

    // get the height/2
    grid_height_2 = grid.height >> 1;

    // just to avoid a lot of divs
    inverse = 1.0/grid.resolution;

    // the laser scan time, used to sync everything
    return ls_scan.time;

}

inline double BeamRangeFinderModel::prob(float real_ray, float sim_ray) {

    // probability parameters
    static double pHit, pShort, pMax, pRand;

    // zhit
    pHit = std::exp(sigma_den*(std::pow((real_ray - sim_ray), 2)));

    // zshort
    if ((0 <= real_ray) && (real_ray <= sim_ray)){

        pShort = (1.0 / (1.0 - (exp(-lambda_short * sim_ray)))) * lambda_short * exp(-lambda_short * real_ray);

    } else {

        pShort = 0.0;

    }

    // zmax
    if(ls_scan.range_max == real_ray){

        pMax = z_max;

    } else {

        pMax = 0.0;

    }

    //zrand
    if((0 <= real_ray) && (ls_scan.range_max >= real_ray)){

        pRand = p_rand;

    } else {

        pRand = 0;

    }

    // returns the probability
    return (z_hit*pHit + z_short*pShort + z_max*pMax + z_rand*pRand);

}
