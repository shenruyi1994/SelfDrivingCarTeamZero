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
        public: std::vector<cv::Point2d> vanishPoint(cv::Mat m, int mid);
				public: double getAngle(double firstX, double firsty,
																double secondX, double secondY, double previousAngle);

		private: sensors::MultiCameraSensorPtr parentSensor;
		private: event::ConnectionPtr updateConnection;
	};
}

#endif
