#include "sdcVisibleObject.hh"

#include <stdio.h>
#include <vector>
#include <map>
#include <math.h>
#include <array>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <opencv2/opencv.hpp>

#include "dataProcessing.hh"
#include "sdcLidarRay.hh"
#include "sdcCar.hh"

using namespace gazebo;

// initialize variables because sensor plugins could update information on dataProcessing
// so there needs to be an initial
double dataProcessing::carX = 0;
double dataProcessing::carY = 0;
double dataProcessing::carYaw = 0;
std::vector<double>* dataProcessing::frontLidarData = new std::vector<double>();
std::vector<double>* dataProcessing::backLidarData = new std::vector<double>();
std::map<LidarPosition, LidarInfo> dataProcessing::lidarInfo = std::map<LidarPosition, LidarInfo>();
cv::Point2d waypoint1;
cv::Point2d waypoint2;
cv::Point2d waypoint3;
cv::Point2d passPoint;
double passPointAngle;
double waypointAngle1;
double waypointAngle2;
double waypointAngle3;
double prev_x;
double prev_y;
double cur_x;
double cur_y;
math::Vector2d unitVector;
bool isNearby_ = false;
float brightness_ = 255;
sdcVisibleObject* dataProcessing::object_;

// When initializing a lidar, store its information such as minimum angle, resoltuion and range
void dataProcessing::InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays) {
  lidarInfo[pos] = LidarInfo(minAngle, resolution, maxRange, numRays);
}

// Update the absolute coordinates of the car in the world
void dataProcessing::UpdateGPS(double x, double y, double z) {
  carX = x;
  carY = y;
  carYaw = z;
}

// Get the car position
cv::Point2d dataProcessing::GetCarLocation() {
  return cv::Point2d(carX, carY);
}

// Update lidar data
void dataProcessing::UpdateLidarData(LidarPosition pos, std::vector<double>* newData) {
  switch (pos) {
    case NEWFRONT:
      frontLidarData = newData;
    break;

    case NEWBACK:
      backLidarData = newData;
    break;

    default:
    break;
  }
}

// Retrieve lidar data
std::vector<double>* dataProcessing::GetLidarData(LidarPosition pos) {
  std::cout << "Get lidar data\n";
  switch (pos) {
    case NEWFRONT:
      return frontLidarData;
    break;

    case NEWBACK:
      return backLidarData;
    break;

    default:
    break;
  }
  return NULL;
}

// called by cameraPluginto update waypoints processed from camera images
void dataProcessing::updateWaypoints(std::vector<cv::Point2d> waypoints) {
  waypoint1 = waypoints[0];
  waypoint2 = waypoints[1];
  waypoint3 = waypoints[2];
}

// called by cameraPlugin to update waypoint angles processed from camera images
void dataProcessing::updateWaypointsAngles(std::vector<double> waypointAngles) {
  waypointAngle1 = waypointAngles[0];
  waypointAngle2 = waypointAngles[1];
  waypointAngle3 = waypointAngles[2];
}

// called by controllers to get waypoints from this dataProcessing platform
std::array<cv::Point2d, 3> dataProcessing::getWaypoints() {
  return { waypoint1, waypoint2, waypoint3 };
}

// called by controllers to get waypoints angles from this dataProcessing platform
std::array<double, 3> dataProcessing::getWaypointAngles() {
  return { waypointAngle1, waypointAngle2, waypointAngle3 };
}

// called by controllers to get nearby obstacle if there is any nearby obstacle
sdcVisibleObject* dataProcessing::GetNearbyObject() {
  return object_;
}

// called by controllers to check if there is any nearby obstacle
bool dataProcessing::IsNearbyObject() {
  return isNearby_;
}

// called by controllers to get obstacle type if there is any obstacle
ObjectType dataProcessing::GetObjectType(const sdcVisibleObject* obj) {
  return obj->GetBrightness() > 100 ? CAR_TYPE : NON_CAR_TYPE;
}

// called by lidarPlugin to update whether there is any obstacle around
void dataProcessing::UpdateIsNearbyObject(bool isNearby) {
  isNearby_ = isNearby;
}

// called by lidarPlugin to update the object information if there is any obstacle around
void dataProcessing::UpdateObject(sdcVisibleObject* obj){
  object_ = obj;
}

// Update car's current and previous positions and
// compute a unit vector of the car's direction
void dataProcessing::UpdateCarDirection(){
  math::Vector2d pos = math::Vector2d(carX, carY);
  double x_coord = pos[0];
  double y_coord = pos[1];

  if(prev_x == 0 && prev_y == 0 && cur_x == 0 && cur_y == 0){
    cur_x = x_coord;
    cur_y = y_coord;
  }else{
    prev_x = cur_x;
    prev_y = cur_y;
    cur_x = x_coord;
    cur_y = y_coord;
    ComputeUnitVector(prev_x, prev_y, cur_x, cur_y);
  }
}

// Compute the unit vector of car's direction
void dataProcessing::ComputeUnitVector(double prev_x, double prev_y, double cur_x, double cur_y){
  double dx = cur_x - prev_x;
  double dy = cur_y - prev_y;
  double mag = GetVectorMagnitude(dx, dy);
  double unit_dx = 0;
  double unit_dy = 0;

  // check for division by 0
  if(mag != 0){
    unit_dx = dx/mag;
    unit_dy = dy/mag;
  }
  unitVector = math::Vector2d(unit_dx, unit_dy);
}

// find angles of the given point to the car
double dataProcessing::FindAngle(double lat_dist, double long_dist){
  return (long_dist != 0) ? atan(lat_dist/long_dist) : PI/2.0;
}

// calculate the magnitude of a given vector
double dataProcessing::GetVectorMagnitude(double x, double y){
  return std::sqrt(x*x + y*y);
}

// compute the vector from the car to the safety target point to avoid the obstacle
math::Vector2d dataProcessing::ComputeObstacleVector(double lat_dist, double long_dist, double angle){
  double mag = GetVectorMagnitude(lat_dist, long_dist);

  // rotate by a given angle
  double x_rot = unitVector[0]*cos(-angle) - unitVector[1]*sin(-angle);
  double y_rot = unitVector[0]*sin(-angle) + unitVector[1]*cos(-angle);

  // scale by magnitude
  double x_scaled = x_rot * mag;
  double y_scaled = y_rot * mag;

  // now take the width of the car into account (how many ever unit vectors you want)
  double x_orthogonal = x_scaled - unitVector[0] * long_dist;
  double y_orthogonal = y_scaled - unitVector[1] * long_dist;
  double mag_orthogonal = GetVectorMagnitude(x_orthogonal, y_orthogonal);

  // check for division by 0
  double x_width = 0;
  double y_width = 0;
  if (mag_orthogonal != 0){
    x_width = x_orthogonal/mag_orthogonal * 1.8;
    y_width = y_orthogonal/mag_orthogonal * 1.8;
  }

  if(angle >= 0){
    x_orthogonal -= x_width;
    y_orthogonal -= y_width;
  }else{
    x_orthogonal += x_width;
    y_orthogonal += y_width;
  }

  // add orthogonal and its perpendicular vectors together
  double newX = x_orthogonal + unitVector[0] * long_dist;
  double newY = y_orthogonal + unitVector[1] * long_dist;

  return math::Vector2d(newX, newY);
}

// get the coordinates of the obstacle for avoidance.
cv::Point2d dataProcessing::getObstacleCoords(){
  sdcVisibleObject* object = GetNearbyObject();

  // get left ray
  sdcLidarRay left = object->getLeftRay();

  // lateral and longitudinal distance to the left edge
  double left_lat = left.GetLateralDist();
  double left_long = left.GetLongitudinalDist();
  double left_angle = FindAngle(left_lat, left_long);

  // find the vector to the left side of the obstacle
  math::Vector2d left_vector = ComputeObstacleVector(left_lat, left_long, left_angle);
  cv::Point2d leftP = cv::Point2d(cur_x+left_vector[0], cur_y+left_vector[1]);

  return leftP;
}

// called by cameraPlugin to update the pass point for the car
void dataProcessing::UpdatePassPoint(cv::Point2d point){
  passPoint = point;
}

// called by controllers get the passing point for the car
cv::Point2d dataProcessing::GetPassPoint(){
  return passPoint;
}

// called by cameraPlugin to update the angle for the car to get to the passing point
void dataProcessing::UpdatePassPointAngle(double theta){
  passPointAngle = theta;
}

// called by controllers to get the angle for the car to get to the passing point
double dataProcessing::GetPassPointAngle(){
  return passPointAngle;
}

/*
 * Get the current sensor readings for position
 */
math::Vector2d dataProcessing::GetPosition() {
    return math::Vector2d(carX, carY);
}

/*
 * Get the current sensor readings for orientation
 */
sdcAngle dataProcessing::GetYaw() {
    return carYaw;
}
