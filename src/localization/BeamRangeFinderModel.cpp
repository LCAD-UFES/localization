#include "BeamRangeFinderModel.hpp"
#include <math.h>


// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m), laser_update(), map_update() {
    // get the z_hit parameter
    private_nh.param("likelihood_z_hit", z_hit, 0.9);
    // get the z_short parameter
    private_nh.param("likelihood_z_short", z_short, 0.9);
    // get the z_max parameter
    private_nh.param("likelihood_z_max", z_max, 0.05);
    // get the z_rand parameter
    private_nh.param("likelihood_z_rand", z_rand, 0.05);
    // get the sigma_hit parameter
    private_nh.param("likelihood_sigma_hit", sigma_hit, 0.2);
    // get the lambda short
    private_nh.param("likelihood_lambda_short", lambda_short, 0.2);

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
      int numRanges = 360;
      //double z;
      double q = 1;
      double p;
      //double phit;
    //double *pose = sample->pose.v;
    sensor_msgs::LaserScan::Ptr ray_casting;
      //w = 1;
      p = 0;

      //ray_casting com os parametros do laser
      ray_casting = occupancy_grid_utils::simulateRangeScan(map_update, convertToPose(sample), laser_update, false);


      //calculo da probabilidade de cada beam
      for(int zt = 0; zt<numRanges; zt+=step){
          double pHit, pShort, pMax, pRand = 0;
          double ztk = laser_update.ranges[zt];
          double ztk_star = ray_casting->ranges[zt];

          //pHit = (1/(sqrt(2 * M_PI * pow(sHit,2)))) * exp(-0.5*((pow((zkt-zkt_star),2))/pow(sHit,2)));
          //if?

          //zhit
          pHit = exp(-0.5*((pow((ztk-ztk_star),2))/sigma_hit2));
         // ROS_INFO("hit: %f", pHit);
          //        z = laseri.ranges[zt] - ray_casting->ranges[zt];
          //        p += zhit*(pre_calculo_hit*(exp(-(z * z) / (2 * sigma_hit * sigma_hit))));

          //zshort
          if ((0 <= ztk) && (ztk <= ztk_star)){
              //p+= zshort * (1/(1-exp(-lambda_short*ztk_star))) * (lambda_short*exp(-lambda_short*ztk));
              pShort = (1.0 / (1.0 - (exp(-lambda_short * ztk_star)))) * lambda_short * exp(-lambda_short * ztk);
          }
         // ROS_INFO("short: %f", pShort);
          //zmax
          if(ztk == 30.0){
              pMax=z_max*1.0;
          }else{
              pMax = 0;
          }
       //   ROS_INFO("max: %f", pMax);
        //  ROS_INFO("ZTK: %f", ztk);
          //zrand
          if((0<=ztk) && (ztk<=30)){
              pRand=1.0/30;
          }else{
              pRand = 0;
          }
        //  ROS_INFO("rando: %f\n", pRand);

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
    step = 6;
    if(step<1){
        step = 1;
    }

    // copy the map
    map->getMap(&map_update);

    return ls_scan.time;
}

const geometry_msgs::Pose BeamRangeFinderModel::convertToPose(Sample2D *p){
        geometry_msgs::Pose new_pose;

        // lock the mutex
        //cmds_mutex.lock();

//        // copy the x coord
//        new_pose.position.x = p[0];
//        // copy the y coord
//        new_pose.position.y = p[1];

        tf::poseTFToMsg(
                            tf::Pose(
                                tf::createQuaternionFromYaw(p->pose.v[2]),
                                tf::Vector3(p->pose.v[0], p->pose.v[1], 0)),
                            new_pose
                       );

    const geometry_msgs::Pose &od = new_pose;

    // unlock the mutex
    //cmds_mutex.unlock();

    return od;
}
