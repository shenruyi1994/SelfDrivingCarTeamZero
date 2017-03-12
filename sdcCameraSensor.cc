/*
 * This class registers and updates the front camera sensor.
 *
 * This class also handles video processing and lane finding based
 * upon video inputs to handle lane finding and tracking using
 * methods found in the paper "A lane-curve detection based on LCF"
 */

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "sdcCameraSensor.hh"
#include "fadiff.h"

using namespace fadbad;
using namespace gazebo;
using namespace cv;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcCameraSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

// Cascade Classifier information using CPU
CascadeClassifier cpu_stop_sign;
String cascade_file_path = "OpenCV/haarcascade_stop.xml";

//FADBAD-wrapped forward differentiation for gradient calculations
F<double> delG(const F<double>& x, const F<double>& y) {
	F<double> dG = sqrt(pow(x,2)+pow(y,2));
	return dG;
}

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {
		// Get the parent sensor.
		this->parentSensor =
		boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
		
		// Make sure the parent sensor is valid.
		if (!this->parentSensor) {
				gzerr << "Couldn't find a camera\n";
				return;
		}

		// Connect to the sensor update event.
		this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcCameraSensor::OnUpdate, this));

		// Make sure the parent sensor is active.
		this->parentSensor->SetActive(true);
		//std::cout << this->parentSensor->GetNoise() << std::endl;
		if (!cpu_stop_sign.load(cascade_file_path)) {
			std::cout << "Unable to load cascade classifier xml file!" << std::endl;
		}
}

// Called by the world update start event
void sdcCameraSensor::OnUpdate() {
	// Pull raw data from camera sensor object as an unsigned character array with 3 channels.
	const unsigned char* img = this->parentSensor->GetImageData(0);
	Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

	//Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
	//set area for ROI as a rectangle
       
	Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	Mat imageROI = image(ROI);

	// Canny algorithm for edge dectection
	Mat contours, contours_thresh;
	Canny(imageROI,contours,50,150);
	threshold(contours,contours_thresh,127,255, THRESH_BINARY);

	Mat imageGray;
	Mat imageInv = Mat(imageROI.rows, imageROI.cols, CV_8UC1, Scalar(0));
	cvtColor(imageROI,imageGray,CV_BGR2GRAY);
	adaptiveThreshold(imageGray,imageInv, 255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,201, -20 );

// BEGIN STRAIGHT LANE DETECTION
	// Hough Transform detects lines within the edge map, stores result in lines.
	float PI = 3.14159;
	std::vector<Vec2f> lines;
	HoughLines(contours,lines,1,PI/180, 75);

	std::vector<Vec2f>::const_iterator it = lines.begin();

	Vec2f left_lane_marker = Vec2f(0.0, PI);
	Vec2f right_lane_marker = Vec2f(0.0, 0.0);

	while (it!=lines.end()) {
			float rho= (*it)[0];   // first element is distance rho
			float theta= (*it)[1]; // second element is angle theta

				if ( 0 < theta < (PI/2 - 0.1) && theta < left_lane_marker[1]) {
					left_lane_marker = Vec2f(rho,theta);
				}
				if ((PI/2 + 0.1) < theta < (PI - 0.1) && theta > right_lane_marker[1]) {
					right_lane_marker = Vec2f(rho,theta);
				}

			// This code can be uncommented to display all of the lines found with the Hough Line Transform
			// Point pt1(rho/cos(theta),0);
			// Point pt2((rho-imageROI.rows*sin(theta))/cos(theta),imageROI.rows);
			// line(image, pt1, pt2, Scalar(0,0,255), 1);
			++it;
	}

	// ATTN: need to have imageROI.rows in numerator because of trig. It isnt an oversight!
	// Lines are drawn on the image, not the region of interest.

	//draw left lane marker
	Point leftp1 = Point(left_lane_marker[0]/cos(left_lane_marker[1]),0.5*image.rows);
	Point leftp2 = Point((left_lane_marker[0] - (imageROI.rows) * sin(left_lane_marker[1])) / cos(left_lane_marker[1]), (image.rows));
	line(image, leftp1, leftp2, Scalar(255), 3);

	//draw right lane marker
	Point rightp1 = Point(right_lane_marker[0]/cos(right_lane_marker[1]),0.5*image.rows);
	Point rightp2 = Point((right_lane_marker[0] - (imageROI.rows) * sin(right_lane_marker[1])) / cos(right_lane_marker[1]), (image.rows));
	line(image, rightp1, rightp2, Scalar(255), 3);
// END STRAIGHT LANE DETECTION
	// Display results to GUI
	namedWindow("Camera View", WINDOW_AUTOSIZE);
	imshow("Camera View", image);
	waitKey(4);
}
