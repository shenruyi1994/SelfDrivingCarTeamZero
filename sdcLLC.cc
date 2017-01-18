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


  static int counter = 0;

  // if(counter%20000 == 0){
  if(counter == 0) {
    std::vector<Waypoint> testPoints;
  Waypoint testPoint;
  testPoint.x = 10;
  testPoint.y = 0;
  testPoint.direction = 0;

  dubins_ = new dubins();

  testPoints.push_back(testPoint);
  path_ = dubins_->calculateDubins(testPoints);

  cv::Point2d testDubinsPoint =  GetDubinsPoint(15);

  // cv::Point3d testPos = dubins_->rightTurn(0, 0, 0, 0.5);

  //std::cout << "straight turn testy: " << testPos.x << ", " << testPos.y << std::endl;

  }
  counter++;
   


  
  //orientation = car_->sdcCar::GetOrientation();
  
  // angleToTarget = car_->sdcCar::AngleToTarget(target);

  // car_->sdcCar::SetTargetDirection(angleToTarget);
  //sdcHLC will call Match Target Direction on this target angle
  //should probably move MatchTargetDirection into sdcLLC
  //std::cout << "The car is moving in the direction:"  << direction << std::endl;
  //std::cout << "The car is facing the direction:" << orientation << std::endl;
  //std::cout << "Angle between coords of car and target" << angleToTarget << std::endl;


  
  }

sdcLLC::sdcLLC(sdcCar* car): car_(car) {
  
}



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

std::vector<Control> dubinsPointHelper(std::vector<Control> controls, double distance){
  std::vector<Control>::iterator it;

  std::vector<Control> newControls;

  for(it=controls.begin(); it < controls.end(); it++){
    Control temp;
    temp.direction = it->direction;
    if (it->distance >= distance){
      temp.distance = it->distance;
      distance = distance - it->distance;

    }
    else{
      temp.distance = distance;
      distance = 0;
    }
    newControls.push_back(temp);

  }
  return newControls;
}

cv::Point2d sdcLLC::GetDubinsPoint(double distance) const {
  math::Vector2d carPos = sdcSensorData::GetPosition();
  cv::Point3d origin;
  //origin.x = carPos.x;
  //origin.y = carPos.y;
  // origin.z = car_->GetDirection().angle;
  
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  std::vector<Control> cont = dubins_->pathToControls(path_);
  
  std::vector<Control>::iterator it;
  double currentAngle = 0;
  cv::Point3d newPoint;

  cont = dubinsPointHelper(cont, 15);

  for(it=cont.begin(); it < cont.end(); it++){

    switch(it->direction){
    case -1:
      newPoint = dubins_->leftTurn(origin.x, origin.y, origin.z, it->distance);
      origin.x = newPoint.x;		       
      origin.y = newPoint.y;
      origin.z = newPoint.z;
    case 0:
       newPoint = dubins_->straightTurn(origin.x, origin.y, origin.z, it->distance);
      origin.x = newPoint.x;
      origin.y = newPoint.y;
      origin.z = newPoint.z;
    case 1:
       newPoint = dubins_->rightTurn(origin.x, origin.y, origin.z, it->distance);
      origin.x = newPoint.x;
      origin.y = newPoint.y;
      origin.z = newPoint.z;
	}
  }

  std::cout << "OUR NEW POSITION AFTER THE DUBINS IS!!!!: " << origin.x << ", " << origin.y << ", " << origin.z << std::endl;



  //double dist = sqrt(2*MIN_TURNING_RADIUS*MIN_TURNING_RADIUS-2*MIN_TURNING_RADIUS*MIN_TURNING_RADIUS*cos(theta));
  // std::cout << "The distance is: " << dist << "\n";
  
}


