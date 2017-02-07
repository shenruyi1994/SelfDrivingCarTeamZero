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
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
    public: void OnUpdate();
    public: void ROI(cv::Mat &m, int lo, int hi);
    public: cv::Mat preprocess(cv::Mat m);
    public: void updateObjectBrightness(sdcVisibleObject visibleObject);
    public: std::pair<cv::Point2d, cv::Point> vanishPoint(cv::Mat m, int mid);
    public: double getAngle(int firstX, int firsty, int secondX, int secondY,
                            double previousAngle);
    private: sensors::MultiCameraSensorPtr parentSensor;
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
