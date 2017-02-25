#ifndef _CameraPlugin_hh
#define _CameraPlugin_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "dataProcessing.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE CameraPlugin : public SensorPlugin
  {
    public:
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
      void OnUpdate();
      void ROI(cv::Mat &m, int lo, int hi);
      cv::Mat preprocess(cv::Mat m);
      void updateObjectBrightness(sdcVisibleObject* visibleObject);
      void WaypointSearch(int width, int height);
      void ReturnPointSearch(int width, int height);
      std::pair<cv::Point2d, cv::Point> vanishPoint(cv::Mat m, int mid);
      double getAngle(const cv::Point2d& p1, const cv::Point2d& p2);
      static void SetReturnMode(bool mode);
      
      
    private:
      sensors::MultiCameraSensorPtr parentSensor;
      event::ConnectionPtr updateConnection;
  };
}

#endif
