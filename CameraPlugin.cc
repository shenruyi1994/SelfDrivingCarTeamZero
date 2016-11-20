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

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "CameraPlugin.hh"

#define PI 3.14159265359

using namespace gazebo;
using namespace cv;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;


void CameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
	this->parentSensor =
		boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

	if(!this->parentSensor)
	{
		gzerr << "Couldn't find a camera\n";
		return;
	}

	// connect to the sensor update event.
	this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&CameraPlugin::OnUpdate, this));

	// make sure the parent sensor is active
	this->parentSensor->SetActive(true);
}

void CameraPlugin::OnUpdate()
{
	// get image width, height and data
	int width = (int)this->parentSensor->GetImageWidth(0);
	int height = (int)this->parentSensor->GetImageHeight(0);
	const unsigned char* imageData = this->parentSensor->GetImageData(0);

	// create image matrix
	Mat image = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));
	Mat imageBirdsEye = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));


	// use Canny algorithm to apply on the image to get contours. (args: input, output, low threshold, high threshold)
	Mat contours,cdst;
	Canny(image,contours,125,350);
	cvtColor(contours, cdst, CV_GRAY2BGR);
	// invert the image to better show the lines (threshold value below 128 becomes 255)
	Mat contoursInv;
	threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

	// Hough transform for line detection
	//float PI = 3.141592;
	std::vector<Vec2f> lines;
	HoughLines(contoursInv,lines,1,PI/180, 75);

	std::vector<cv::Vec2f>::const_iterator it= lines.begin();

	//vertical angle is 0
	// horizontal angel is 2/pi
	double leftEdgeAngle = 0;
	double rightEdgeAngle = 0;
	double leftRho= 0;
	double rightRho= 0;

	//The part of straight lane detection
	while (it!=lines.end()) {
		double currentRho = (*it)[0];
    double currentAngle = (*it)[1];

    // horizontal lines are 90 degrees and vertical lines are 0 degrees.
    if (0 < currentAngle <= (PI/2) && currentAngle < leftEdgeAngle) {
      leftEdgeAngle = currentAngle;
			leftRho = currentRho;
    } else if (PI/2 <= currentAngle < PI && currentAngle > rightEdgeAngle) {
      rightEdgeAngle = currentAngle;
			rightRho = currentRho;
    }
		//cv::Point pt1(currentRho/cos(currentAngle),0);
		//cv::Point pt2((currentRho-image.rows*sin(currentAngle))/cos(currentAngle),image.rows);
		//cv::line(contoursInv, pt1, pt2, cv::Scalar(143), 1);
		//std::cout << "Pt1 is "<< pt1 << "AND Pt2 is " << pt2 << "\n" << std::endl;
    ++it;
  }

	cv::Point pt1(leftRho/cos(leftEdgeAngle),0);
	cv::Point pt1_1(rightRho/cos(rightEdgeAngle),0);
	// point of intersection of the line with last row
	cv::Point pt2((leftRho-image.rows*sin(leftEdgeAngle))/cos(leftEdgeAngle),image.rows);
	cv::Point pt2_1((rightRho-image.rows*sin(rightEdgeAngle))/cos(rightEdgeAngle),image.rows);
	// draw a gray line
	cv::line(contoursInv, pt1, pt2, cv::Scalar(143), 1);
	cv::line(contoursInv, pt1_1, pt2_1, cv::Scalar(143), 1);


	// Turning angle would be the average angels of left edge and right egde
	double turningAngle = (leftEdgeAngle+rightEdgeAngle)/2;
	double turningRho = (leftRho+rightRho)/2;

	cv::Point pt1_2(turningRho/cos(turningAngle),0);
	cv::Point pt2_2((turningRho-image.rows*sin(turningAngle))/cos(turningAngle),image.rows);
	cv::line(contoursInv, pt1_2, pt2_2, cv::Scalar(143), 1);
	std::cout << "Turning point1 is "<< pt1_1 << "Turning point2 is " << pt2_2 << "\n" << std::endl;


	// Textbook way of prnting hough tranformation lines

	//std::vector<cv::Vec2f>::const_iterator it= lines.begin();
  //     while (it!=lines.end()) {
  //        float rho= (*it)[0];   // first element is distance rho
  //        float theta= (*it)[1]; // second element is angle theta
  //        if (theta < PI/4.
  //             || theta > 3.*PI/4.) { // ~vertical line
  //           // point of intersection of the line with first row
  //           cv::Point pt1(rho/cos(theta),0);
  //           // point of intersection of the line with last row
  //           cv::Point pt2((rho-image.rows*sin(theta))/
  //                                    cos(theta),image.rows);
  //           // draw a white line
  //           cv::line(contoursInv, pt1, pt2, cv::Scalar(255), 1);
  //        } else { // ~horizontal line
  //           // point of intersection of the
  //           // line with first column
  //           cv::Point pt1(0,rho/sin(theta));
  //           // point of intersection of the line with last column
  //           cv::Point pt2(image.cols,
  //              (rho-image.cols*cos(theta))/sin(theta));
  //     // draw a white line
  //     cv::line(contoursInv, pt1, pt2, cv::Scalar(255), 1);
	// 		std::cout << "P1 is "<< pt1 << "AND P2 is " << ((*it)[2],(*it)[3]) << pt2 << "\n" << std::endl;
	 //
  //  }
	//  ++it; }

		// cv::Scalar color=cv::Scalar(255,255,255);
		// while (it!=lines.end()) {
    //           cv::Point pt1((*it)[0],(*it)[1]);
    //           cv::Point pt2((*it)[2],(*it)[3]);
		//
    //           cv::line(contoursInv, pt1, pt2, color);
		// ++it; }
		// it = lines.begin();
		//imshow("source", contoursInv);

	// // Update our steering based on the lane markers
	// double angle_average = (left_lane_marker[1]+right_lane_marker[1]) /2;
	// double newSteeringAmount = -20*PI*(fabs(angle_average - PI/2))+50;
	// sdcSensorData::UpdateSteeringMagnitude(newSteeringAmount);

	//////////////// Perspective Transform ////////////////////
	///// adapted from http://stackoverflow.com/questions/7838487/executing-cvwarpperspective-for-a-fake-deskewing-on-a-set-of-cvpoint ///////

	//find actual points based on hough lines? for now these are hardcoded
    vector<Point> trapezoid_view_points;
    trapezoid_view_points.push_back(Point((width-1)/3, (height-1)/2));
    trapezoid_view_points.push_back(Point(((width-1)/3)*2, (height-1)/2));
    trapezoid_view_points.push_back(Point(width-1, ((height-1)/3)*2));
    trapezoid_view_points.push_back(Point(0, ((height-1)/3)*2));


    const Point* point = &trapezoid_view_points[0];
    int n = (int)trapezoid_view_points.size();
    Mat draw = contoursInv.clone();
    polylines(draw, &point, &n, 1, true, Scalar(0, 0, 0), 3, CV_AA); //CV_AA is anti-aliasing flag
    // save this for debugging
    // imwrite("draw.jpg", draw);

    // Assemble a rotated rectangle out of that info
    RotatedRect box = minAreaRect(cv::Mat(trapezoid_view_points));
    // std::cout << "Rotated box set to (" << box.boundingRect().x << "," << box.boundingRect().y << ") " << box.size.width << "x" << box.size.height << std::endl;

    Point2f pts[4];

    box.points(pts);

    cv::Point2f src_vertices[3];
    src_vertices[0] = pts[0];
    src_vertices[1] = pts[1];
    src_vertices[2] = pts[3];
    //src_vertices[3] = trapezoid_view_points[3];

    Point2f dst_vertices[3];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(box.boundingRect().width-1, 0);
    dst_vertices[2] = Point(0, box.boundingRect().height-1);

   /* Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);

    cv::Mat rotated;
    cv::Size size(box.boundingRect().width, box.boundingRect().height);
    warpPerspective(src, rotated, warpMatrix, size, INTER_LINEAR, BORDER_CONSTANT);*/
    Mat warpAffineMatrix = getAffineTransform(src_vertices, dst_vertices);

    cv::Mat rotated;
    cv::Size size(box.boundingRect().width, box.boundingRect().height);
    warpAffine(contoursInv, rotated, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);

    cv::transpose(rotated, rotated);
    cv::flip(rotated, rotated, 0);

    // imwrite("rotated.jpg", rotated);
    //namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
    //imshow("CameraView", rotated);
	/////////////// end perspective transform //////////////

	/////////// feature detection /////////////////////
    ////// from http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_detection/feature_detection.html  ///////////

	int minHessian = 300;//1200; // higher means fewer points detected

	SurfFeatureDetector detector( minHessian );
	std::vector<KeyPoint> keypoints_1; //, keypoints_2;

	int numChunks = 3; // number of waypoints
	std::vector<Point2f> waypointPoints;
	detector.detect( contoursInv, keypoints_1 );

	std::cout<< "Keypoint vector length: " << keypoints_1.size() << std::endl;
	if (keypoints_1.size() > 0) {
		std::cout << "Window dimensions: " << width << ", " << height << std::endl;

		float roiMinCutoff = height/2; // bounds roi to bottom half of image
		float roiMaxCutoff = height - (height/3); // bounds roi on bottom 
		float increment = (roiMaxCutoff - roiMinCutoff)/numChunks;
		float rightWeightScale = 0.05;
		for (int i = 0; i < numChunks; i++) {
			float xSum = 0;
			float ySum = 0;
			float curMin = roiMinCutoff+(i*increment);
			float curMax = curMin + increment;
			int count = 0;
			for (int j = 0; j < keypoints_1.size(); j++) {
				if (keypoints_1[j].pt.y >= curMin && keypoints_1[j].pt.y <= curMax) {
					xSum += keypoints_1[j].pt.x+rightWeightScale*width;
					ySum += keypoints_1[j].pt.y;
					count++;
				}
			}
			cv::Point2f newWaypoint(xSum/count, ySum/count);
			waypointPoints.push_back(newWaypoint);
			std::cout << "New waypoint: " << newWaypoint.x << ", " << newWaypoint.y << std::endl;

		}

		//-- Draw keypoints
		Mat img_keypoints_1;
		//uncomment to show feature points (also need to replace contoursInv below this with img_keypoints_1):
		//drawKeypoints( contoursInv, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

		int thickness = 2;
		int lineType = 8;
		std::cout << "waypointPoints.size() : " << waypointPoints.size() << std::endl;

		int pointThickness = -1;
		int pointRadius = 6;
		//ineType = 8;

		//print waypoints:
		for (int i = 0; i < waypointPoints.size() - 1; i++) {
			// std::cout << "Point to print: " << waypointPoints[i].x << ", " << waypointPoints[i].y << std::endl;
			// std::cout << "Point to print: " << waypointPoints[i+1].x << ", " << waypointPoints[i+1].y << std::endl;
			circle( contoursInv, waypointPoints[i], pointRadius, Scalar( 0, 0, 255 ), pointThickness, lineType );
			line( contoursInv, waypointPoints[i], waypointPoints[i+1], Scalar( 200, 100, 0 ), thickness, lineType );
		}

		circle( contoursInv, waypointPoints[waypointPoints.size() - 1], pointRadius, Scalar( 0, 0, 255 ), pointThickness, lineType );



		//-- Show detected keypoints
		imshow("Keypoints 1", contoursInv );
	}


	//////////// end feature detection /////////////////


	// camera view display
	//namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
	//imshow("Camera View", contoursInv);

	// update way point data
	dataProcessing::UpdateCameraData(0, 0);

	// camera display
	// imshow("Contour Inverse", contoursInv);
	// imshow("Contour", contours);
	waitKey(4);
}
