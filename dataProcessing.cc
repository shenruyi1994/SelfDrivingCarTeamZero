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
#include "sdcCar.hh"

using namespace gazebo;

double dataProcessing::carX = 0;
double dataProcessing::carY = 0;
double dataProcessing::carZ = 0;
std::vector<double>* dataProcessing::frontLidarData = new std::vector<double>();
std::vector<double>* dataProcessing::backLidarData = new std::vector<double>();
std::map<LidarPosition, LidarInfo> dataProcessing::lidarInfo = std::map<LidarPosition, LidarInfo>();
cv::Point2d waypoint1;
cv::Point2d waypoint2;
cv::Point2d waypoint3;
double waypointAngle1;
double waypointAngle2;
double waypointAngle3;
bool areNearby = false;
float brightness_ = 255;
std::vector<sdcVisibleObject*> dataProcessing::objectList_ = std::vector<sdcVisibleObject*>();

// When initializing a lidar, store its information such as minimum angle, resoltuion and range
void dataProcessing::InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays) {
  lidarInfo[pos] = LidarInfo(minAngle, resolution, maxRange, numRays);
}

// Update the absolute coordinates of the car in the world
void dataProcessing::UpdateCarPosition(double x, double y, double z) {
  carX = x;
  carY = y;
  carZ = z;
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
  //std::cout << "Update(Lidar)\n";  // check for update
}

void dataProcessing::GetLanePosition() {
  std::cout << "Get lane position data\n";
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

void dataProcessing::updateWaypoints(std::vector<cv::Point2d> waypoints) {
  waypoint1 = waypoints[0];
  waypoint2 = waypoints[1];
  waypoint3 = waypoints[2];
}

void dataProcessing::updateWaypointsAngles(std::vector<double> waypointAngles) {
  waypointAngle1 = waypointAngles[0];
  waypointAngle2 = waypointAngles[1];
  waypointAngle3 = waypointAngles[2];
}

std::array<cv::Point2d, 3> dataProcessing::getWaypoints() {
  return { waypoint1, waypoint2, waypoint3 };
}

std::array<double, 3> dataProcessing::getWaypointAngles() {
  return { waypointAngle1, waypointAngle2, waypointAngle3 };
}

std::vector<sdcVisibleObject*> dataProcessing::GetNearbyObjects() {
  return objectList_;
}

bool dataProcessing::AreNearbyObjects() {
  return areNearby;
}

ObjectType dataProcessing::GetObjectType(const sdcVisibleObject* obj) {
  return obj->GetBrightness() > 100 ? CAR_TYPE : NON_CAR_TYPE;
}

void dataProcessing::UpdateAreNearbyObjects(bool areNearby) {
  areNearby = areNearby;
}

void dataProcessing::UpdateObjectList(std::vector<sdcVisibleObject*> objs){
  objectList_ = objs;
}
