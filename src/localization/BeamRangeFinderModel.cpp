#include "BeamRangeFinderModel.hpp"
#include <cmath>
#include <math.h>
#include <time.h>


// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) :
    MeasurementModel(ls, m), laser_update(), map_update() {

    // get the z_hit parameter
    private_nh.param("beam_z_hit", z_hit, 0.7);
    // get the z_short parameter
    private_nh.param("beam_z_short", z_short, 0.2);//
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

    //pre-computing a few things

    sigma_hit2 = sigma_hit*sigma_hit;
    //part_gaussian = 1/sqrt(2*M_PI*sigma_hit2);
    z_random_max = z_rand/30.0;

}

//
double BeamRangeFinderModel::getWeight(Sample2D *sample) {

    //verificar o numero de beam!!!
    int numRanges = laser_update.ranges.size();
    //double z;
    double q = 1;
    double p;
   //get the origin in map

    sensor_msgs::LaserScan::Ptr ray_casting;
      //w = 1;
      p = 0;

    ray_casting = occupancy_grid_utils::simulateRangeScan(map_update, convertToPose(sample), laser_update, false);

    //calculo da probabilidade de cada beam
    for(int zt = 0; zt<numRanges; zt+=step){
        double pHit, pShort, pMax, pRand = 0;
        double ztk = laser_update.ranges[zt];
        double ztk_star = ray_casting->ranges[zt];

        //zhit
        pHit = exp(-0.5*((pow((ztk-ztk_star),2))/sigma_hit2));

        //zshort
        if ((0 <= ztk) && (ztk <= ztk_star)){

            pShort = (1.0 / (1.0 - (exp(-lambda_short * ztk_star)))) * lambda_short * exp(-lambda_short * ztk);

        }

        if(ztk == 30.0){
            pMax=z_max*1.0;
        }else{
            pMax = 0;
        }

        //zrand
        if((0<=ztk) && (ztk<=30)){
            pRand=1.0/30;
        }else{
            pRand = 0;
        }


        p = z_hit * pHit + z_short * pShort + z_max * pMax + z_rand * pRand;
        //if?
        q = q* p;
    }
    sample->weight = q;
    // save the weight

    return sample->weight;
    // ROS_INFO("peso: %f\n", q);

}


// update the LaserScan
ros::Time BeamRangeFinderModel::update() {

    // copy the laser
    laser->getLaserScan(&laser_update);

    step = laser_update.ranges.size()/(max_beams - 1);

    // verify the step
    if(step<1){
        step = 1;
    }

    // copy the map
    map->getMap(&map_update);

    return laser_update.header.stamp;
}

const geometry_msgs::Pose BeamRangeFinderModel::convertToPose(Sample2D *p){

   //lock mutex
   // beam_mutex.lock();

    geometry_msgs::Pose new_pose;

    tf::poseTFToMsg(
                        tf::Pose(
                            tf::createQuaternionFromYaw(p->pose.v[2]),
                            tf::Vector3(p->pose.v[0], p->pose.v[1], 0)),
                        new_pose
                    );

    const geometry_msgs::Pose &od = new_pose;
    //unlock mutex
    //beam_mutex.unlock();
    return od;

}
