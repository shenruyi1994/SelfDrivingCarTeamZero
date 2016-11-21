#ifndef _dataProcessing_hh
#define _dataProcessing_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include "LidarInfo.hh"
#include "Waypoints.hh"


namespace gazebo
{
	enum LidarPosition {NEWFRONT, NEWBACK};
	class dataProcessing
	{
		public:
			static void InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays);
			static void UpdateCameraData(double newX, double newY);
			static void UpdateLidarData(LidarPosition pos, std::vector<double>* newData);
			static void GetLanePosition();
			static std::vector<double>* GetLidarData(LidarPosition pos);
	  	static std::vector<cv::Point> getWaypoints();
			static void updateWaypoints(cv::Point p1, cv::Point p2, cv::Point p3);
			static void UpdateCarPosition(double x, double y, double z);
		private:
			static double carX;
			static double carY;
			static double carZ;
			static double lanePositionX;
			static double lanePositionY;
			static std::vector<double>* frontLidarData;
			static std::vector<double>* backLidarData;
			static std::map<LidarPosition, LidarInfo> lidarInfo;
	};
}
#endif
