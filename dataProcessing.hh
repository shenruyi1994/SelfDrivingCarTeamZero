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
      static void UpdateCarPosition(double x, double y, double z);
      static std::vector<sdcVisibleObject*> GetNearbyObjects();
      static bool AreNearbyObjects();
      static ObjectType GetObjectType(const sdcVisibleObject* obj);
      static void UpdateAreNearbyObjects(bool areNearby);
      static void UpdateObjectList(std::vector<sdcVisibleObject*> objs);

    private:
      static double carX;
      static double carY;
      static double carZ;
      static std::vector<double>* frontLidarData;
      static std::vector<double>* backLidarData;
      static std::map<LidarPosition, LidarInfo> lidarInfo;
      static std::vector<sdcVisibleObject*> objectList_;
  };
}
#endif
