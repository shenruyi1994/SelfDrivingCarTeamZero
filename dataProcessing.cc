#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <vector>
#include <map>
#include <math.h>
#include "dataProcessing.hh"
#include "sdcCar.hh"
#include <opencv2/opencv.hpp>

using namespace gazebo;

double dataProcessing::carX = 0;
double dataProcessing::carY = 0;
double dataProcessing::carZ = 0;
double dataProcessing::lanePositionX = 0;
double dataProcessing::lanePositionY = 0;
std::vector<double>* dataProcessing::frontLidarData = new std::vector<double>();
std::vector<double>* dataProcessing::backLidarData = new std::vector<double>();
std::map<LidarPosition, LidarInfo> dataProcessing::lidarInfo = std::map<LidarPosition, LidarInfo>();
cv::Point waypoint1;
cv::Point waypoint2;
cv::Point waypoint3;


// When initializing a lidar, store its information such as minimum angle, resoltuion and range
void dataProcessing::InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays) {
	lidarInfo[pos] = LidarInfo(minAngle, resolution, maxRange, numRays);
}

// Update way point after image processing
void dataProcessing::UpdateCameraData(double newLanePositionX, double newLanePositionY) {
    lanePositionX = newLanePositionX;
    lanePositionY = newLanePositionY;
	//std::cout << "Update(Camera)\n";	// check for update
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
	//std::cout << "Update(Lidar)\n";	// check for update
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

void dataProcessing::updateWaypoints(cv::Point p1, cv::Point p2, cv::Point p3) {
	waypoint1 = p1;
	waypoint2 = p2;
	waypoint3 = p3;
}

std::vector<cv::Point> dataProcessing::getWaypoints() {
	std::vector<cv::Point> waypoints;

  waypoints.push_back(waypoint1);
  waypoints.push_back(waypoint2);
  waypoints.push_back(waypoint3);
  return waypoints;
}
