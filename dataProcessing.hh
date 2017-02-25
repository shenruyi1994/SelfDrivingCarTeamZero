#ifndef _dataProcessing_hh
#define _dataProcessing_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <array>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

#include "globals.hh"
#include "LidarInfo.hh"
#include "sdcLidarRay.hh"
#include "sdcVisibleObject.hh"
#include "Waypoints.hh"

namespace gazebo
{
  enum LidarPosition {NEWFRONT, NEWBACK};

  class dataProcessing
  {
    public:
      static void InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays);
      static void UpdateLidarData(LidarPosition pos, std::vector<double>* newData);
      static void GetLanePosition();
      static std::vector<double>* GetLidarData(LidarPosition pos);
      static std::array<cv::Point2d, 3> getWaypoints();
      static std::array<double, 3> getWaypointAngles();
      static void updateWaypoints(std::vector<cv::Point2d> waypoints);
      static void updateWaypointsAngles(std::vector<double> waypointAngles);
      static void UpdateGPS(double x, double y, double z);
      static cv::Point2d GetCarLocation();
      static sdcVisibleObject* GetNearbyObject();
      static bool IsNearbyObject();
      static ObjectType GetObjectType(const sdcVisibleObject* obj);
      static void UpdateIsNearbyObject(bool isNearby);
      static void UpdateObject(sdcVisibleObject* obj);
      static cv::Point2d getObstacleCoords();
      static void UpdateCarDirection();
      static void ComputeUnitVector(double prev_x, double prev_y, double cur_x, double cur_y);
      static double FindAngle(double lat_dist, double long_dist);
      static math::Vector2d ComputeObstacleVector(double lat_dist, double long_dist, double angle);
      static double GetVectorMagnitude(double x, double y);
      static math::Vector2d getCarVector();
        
    private:
      static double carX;
      static double carY;
      static double carYaw;
      static std::vector<double>* frontLidarData;
      static std::vector<double>* backLidarData;
      static std::map<LidarPosition, LidarInfo> lidarInfo;
      static sdcVisibleObject* object_;
  };
}
#endif
