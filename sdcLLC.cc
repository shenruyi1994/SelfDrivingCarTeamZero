
#include "sdcLLC.hh"
#include "sdcCar.hh"
#include <tuple>

//Typedef placeholder as I dont know how the trajectory will look 
typedef <vector<int> > trajectory_t;
typedef <tuple<double, double, double> > configuration_t;


void llC(){
  Waypoints waypoints;
  trajectory_t optimalTrajectory  = calculateDubins(waypoints);
  this->setTargetSteering();
  this-setTargetSpeed();
}

trajectory_t  calculateDubins(Waypoints waypoints){
  configuration_t initial;
  configuration_t final;

  trajectory_t INFtrajectory;
  trajectory_t csc;
  trajectory_t ccc;

  this->x;
  this->y;
  this->sdcAngle;
  
  return minPath(CSC(inital,final),CCC(initial,final),INFtrajectory, INFtrajectory);

}


trajectory_t CSC(configuration_t initial, configuration_t final) {

  trajectory_t RSRtrajectory;
  trajectory_t LSRtrajectory;
  trajectory_t RSLtrajectory;
  trajectory_t LSRtrajectory;
  
  return minPath(RSRtrajectory, LSRtrajectory, RSLtrajectory, LSRtrajectory);
}

trajectory_t CCC(configuration_t initial, configuration_t final) {

  trajectory_t RLRtrajectory;
  trajectory_t LRLtrajectory;
  trajectory_t INFtrajectory;

  return minPath(RLRtrajectory, LRLtrajectory, INFtrajectory, INFtrajectory);
}

double minPath(trajectory_t trajectory1, trajectory_t trajectory2, trajectory_t trajectory3, trajectory_t trajectory4) {
  trajectory_t minTrajectory;
  return minTrajectory;
}








}
