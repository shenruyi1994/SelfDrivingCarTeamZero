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
  //car_->SetTargetSteeringAmount(0);
  car_->SetTargetSpeed(10);
 
  static int counter = 0;

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
  }

  counter++;
}

// LLC Constructor
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

//Helper function for calculating our dubins point
//Given a distance to travel and input controls corresponding to a dubins path, output controls coresponding to travel a given distance along the path 
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

//Function that finds a point along our dubins path at a specified distance
cv::Point2d sdcLLC::GetDubinsPoint(double distance) const {
  math::Vector2d carPos = sdcSensorData::GetPosition();
  cv::Point3d origin;

  //origin.x = carPos.x;
  //origin.y = carPos.y;
  //origin.z = car_->GetDirection().angle;
  
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;

  std::vector<Control> cont = dubins_->pathToControls(path_);
  
  std::vector<Control>::iterator it;
  double currentAngle = 0;
  cv::Point3d newPoint;
  
  //generates temporary set of controls used to help find a point on the dubins path
  cont = dubinsPointHelper(cont, 15);

  //this loop controls the logic to find a point along our dubins path
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
  std::cout << "Our new position along the dubins path is (x,y,angle) : " << origin.x << ", " << origin.y << ", " << origin.z << std::endl;

  cv::Point2d nullPoint;
  return nullPoint;
}


