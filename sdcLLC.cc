//sdcLLC.hh containsCGAL typedefs and includes - may restructure location
#include "sdcLLC.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <CGAl/Arrangement_2.h>
#include <opencv2/opencv.hpp>

#include "arr_print.h"
#include "dataProcessing.hh"
#include "globals.hh"
#include "sdcCar.hh"
#include "sdcHLC.hh"
#include "Waypoints.hh"
#include "dubins.hh"

using namespace gazebo;

void sdcLLC::update() {
  // std::pair<SteeringAngle, TimeStep> dubins  = calculateDubins(NULL);
  car_->SetTargetSteeringAmount(0);
  car_->SetTargetSpeed(10);
 
  // sdcAngle direction;
  //sdcAngle orientation;
  //sdcAngle angleToTarget;

  //std::vector<cv::Point> waypoints = dataProcessing::getWaypoints();
  //math::Vector2d target(waypoints.front().x, waypoints.front().y);

  Waypoints testPoint;

  
  int control = dubins_->calculateDubins(&testPoint);

  
  //orientation = car_->sdcCar::GetOrientation();
  
  // angleToTarget = car_->sdcCar::AngleToTarget(target);

  // car_->sdcCar::SetTargetDirection(angleToTarget);
  //sdcHLC will call Match Target Direction on this target angle
  //should probably move MatchTargetDirection into sdcLLC
  //std::cout << "The car is moving in the direction:"  << direction << std::endl;
  //std::cout << "The car is facing the direction:" << orientation << std::endl;
  //std::cout << "Angle between coords of car and target" << angleToTarget << std::endl;


  
  }

sdcLLC::sdcLLC(sdcCar* car): car_(car) {}



/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Accelerate(double amt, double rate) {
  car_->SetTargetSpeed(car_->GetSpeed() + amt);
  car_->SetAccelRate(rate);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Brake(double amt, double rate) {
  car_->SetTargetSpeed(car_->GetSpeed() - amt);
  car_->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcLLC::Stop() {
  car_->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired,
 * NOT the direction the front of the car is facing
 */
void sdcLLC::Reverse() {
  car_->reversing_ = true;
}

/*
 * Stop reversing the car.
 */
void sdcLLC::StopReverse() {
  car_->reversing_ = false;
}
