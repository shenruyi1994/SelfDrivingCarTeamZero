#include "sdcLLC.hh"
#include "sdcCar.hh"

void llC(){

  pair<double velocity, double yaw> dubins  = calculateDubins();
  this->setTargetSteering(dubins.first);
  this-setTargetSpeed(dubins.first);
}

pair<double velocity, double yaw>  calculateDubins(Waypoints waypoints){
  this->x;
  this->y;
  this->sdcAngle;
  double velocity;
  double yaw;

  return make_pair(velocity, yaw);

}


}
