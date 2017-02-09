//sdcLLC.hh containsCGAL typedefs and includes - may restructure location
#include "sdcLLC.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
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
  if (car_->model_->GetWorld()->GetSimTime().Double() < 0.2) {
    GenerateNewDubins();
  }
}

sdcLLC::sdcLLC(sdcCar* car): car_(car) {
  dubins_ = new dubins();
  GenerateNewDubins();
}

bool sdcLLC::BeyondPath(double distance) const {
  double totalPathLength = 0;
  for (Path path : paths_) {
    totalPathLength += path.length;
  }
  return distance > totalPathLength;
}

void sdcLLC::GenerateNewDubins() {
  math::Vector2d carPos = sdcSensorData::GetPosition();
  Waypoint carPoint;
  carPoint.x = carPos.x;
  carPoint.y = carPos.y;
  carPoint.direction = car_->GetDirection().angle;
  //printf("initDirection  %f", carPoint.direction);
  paths_.clear();

  std::array<cv::Point2d, 3> rawWaypoints = dataProcessing::getWaypoints();
  std::array<double, 3> waypointAngles = dataProcessing::getWaypointAngles();
  Waypoint startPoint = carPoint;
  for (int i = 0; i < 3; i++) {
    Waypoint waypoint;
    waypoint.x = rawWaypoints[i].x;
    waypoint.y = rawWaypoints[i].y;
    waypoint.direction = waypointAngles[i];

    printf("==========\n  ========== waypoint: (%f, %f)", waypoint.x, waypoint.y);

    paths_.push_back(dubins_->calculateDubins(waypoint, startPoint));
    startPoint = waypoint;
  }
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
std::vector<Control> dubinsPointHelper(std::vector<Control> controls, double distance) {

  std::vector<Control>::iterator it;
  std::vector<Control> newControls;

  for(it = controls.begin(); it < controls.end(); it++) {
    Control temp;
    temp.direction = it->direction;
    if (it->distance <= distance) {
      temp.distance = it->distance;
      distance = distance - it->distance;
    }
    else{
      temp.distance = distance;
      distance = 0;
    }
    newControls.push_back(temp);
    //printf("\n new control(dist, type), %f, %d", temp.distance, temp.direction);
  }
  return newControls;
}

// Finds a point along our dubins path at a specified distance
cv::Point2d sdcLLC::GetDubinsPoint(double distance) {
  GenerateNewDubins();
  Path path;
  if (distance <= paths_[0].length) {
    path = paths_[0];
  } else if (distance <= paths_[0].length + paths_[1].length) {
    path = paths_[1];
    distance -= paths_[0].length;
  } else {
    path = paths_[2];
    distance -= paths_[0].length + paths_[1].length;

    // Here we should probably eventually handle this with slowing down
    distance = fmin(distance, paths_[2].length);
  }

  math::Vector2d carPos = sdcSensorData::GetPosition();
  cv::Point3d origin = cv::Point3d(path.origin);

  std::vector<Control> cont = dubins_->pathToControls(path);

  //cv::Point3d origin (0,0,0);

  //generates temporary set of controls used to help find a point on the dubins path
  cont = dubinsPointHelper(cont, distance);

  //this loop controls the logic to find a point along our dubins path
  for(std::vector<Control>::iterator it = cont.begin(); it < cont.end(); it++) {
    switch(it->direction) {
    case -1:
      origin = dubins_->leftTurn(origin.x, origin.y, origin.z, it->distance);
      break;
    case 0:
      origin = dubins_->straightTurn(origin.x, origin.y, origin.z, it->distance);
      break;
    case 1:
      origin = dubins_->rightTurn(origin.x, origin.y, origin.z, it->distance);
      break;
    }
  }

  // move target point to the origin of our original dubins path
  // tempPoint.x = origin.x - path.origin.x;
  // tempPoint.y = origin.y - path.origin.y;

  //tempPoint.x = 0;
  //temPoint

  //rotate target point around dubins path origin
  //finalPoint.x = origin.x * cos(path.rotationAngle) - origin.y * sin(path.rotationAngle);
  //finalPoint.y = origin.x * sin(path.rotationAngle) + origin.y * cos(path.rotationAngle);

  // scale rotated point back to a place corressponding to its original coords
  // finalPoint.x += path.origin.x;
  // finalPoint.y += path.origin.y;


  printf("(x,y,theta): (%f, %f, %f)\n", origin.x, origin.y, origin.z);
  cv::Point2d returnP;
  returnP.x = origin.x;
  returnP.y = origin.y;
  return returnP;
}
