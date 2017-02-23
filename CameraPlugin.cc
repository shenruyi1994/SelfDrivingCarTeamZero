#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>
#include <math.h>

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

Mat image;
vector<Point2d> sidePoints;

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

  // added to check about the viewport
  int viewportWidth = (int)this->parentSensor->GetCamera(0)->GetViewportWidth();
  int viewportHeight = (int)this->parentSensor->GetCamera(0)->GetViewportHeight();
  // cout << "viewportWidth pixels is " << viewportWidth << endl;
  // cout << "viewportHeight pixels is " << viewportHeight << endl;

  // math::Vector3 originCoord;
  // math::Vector3 direction;
  // this->parentSensor->GetCamera(0)->GetCameraToViewportRay(200, 200, originCoord, direction);
  //
  // // cout << "viewportWidth originCoord is " << originCoord << endl;
  // // cout << "viewportWidth direction is " << direction << endl;
  // double directionx = direction[0];
  // double directiony = direction[1];
  // double directionz = direction[2];

  // create image matrix
  image = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));


    // Rectangular region of interest
    double ROI_lo = 40;
    double ROI_hi = height;

    Mat rect_roi(image.size(), image.type());
    image.copyTo(rect_roi);
    ROI(rect_roi, ROI_lo, ROI_hi);


    // Create sub ROIs
    int n_sub = 3;
    double interval = (ROI_hi-ROI_lo)/2;
    double lo_increment = interval/2;

    double sub_lo = ROI_lo;
    double sub_hi;

    vector<Mat> subs;

    for(int i = 0; i < n_sub; i++)
    {
        Mat sub(rect_roi.size(), rect_roi.type());
        rect_roi.copyTo(sub);
        sub_hi = sub_lo + interval;
        ROI(sub, sub_lo, sub_hi);
        subs.push_back(sub);
        sub_lo += lo_increment;
    }

    // --------------------------------------
    // EDITED ROI III
    /*
    for(int i = 0; i < n_sub-1; i++)
    {
        Mat sub(rect_roi.size(), rect_roi.type());
        rect_roi.copyTo(sub);
        sub_hi = sub_lo + interval;
        ROI(sub, sub_lo, sub_hi);
        subs.push_back(sub);
        sub_lo = sub_hi;
    }

    sub_hi = sub_lo + interval;
    sub_lo -= 220;
    Mat sub(rect_roi.size(), rect_roi.type());
    rect_roi.copyTo(sub);
    ROI(sub, sub_lo, sub_hi);
    subs.push_back(sub);
     */
    // EDITED ROI I
    // --------------------------------------

    //  Process each sub ROI
    vector<Mat> proc_subs;
    for(size_t i = 0; i < subs.size(); i++)
    {
        proc_subs.push_back(preprocess(subs[i]));
    }

    // For each sub ROI, find vanishing point
    vector<cv::Point2d> pts;
    vector<cv::Point2d> worldPts;
    vector<cv::Point> imagePts;
    vector<double> waypointAngles;

    for(size_t i = 0; i < proc_subs.size(); i++)
    {
        int lo = ROI_lo + i * lo_increment;
        int hi = lo + interval;

        std::pair<cv::Point2d, cv::Point> pts = vanishPoint(proc_subs[i], lo);
        worldPts.push_back(std::get<0>(pts));
        imagePts.push_back(std::get<1>(pts));

        circle(image, std::get<1>(pts), 2, Scalar(255,0,0), 3);
    }

    // print out side points
    for(size_t i = 0; i < sidePoints.size(); i++)
    {
        circle(image, sidePoints[i], 2, Scalar(0,0,255), 3);
    }
    sidePoints.clear();

    cv::Point originaPoint = cv::Point2d(320,400);
    imagePts.push_back(originaPoint);

    for(int i = 3; i > 0; i--)
    {
        double angle = getAngle(worldPts[i], worldPts[i-1]);
        // std::cout << "angle " << i <<" is " << angle << '\n';
        waypointAngles.push_back(angle);
    }

    double newAngleOne = waypointAngles[2];
    double newAngleThree = waypointAngles[0];
    waypointAngles[0] = newAngleOne;
    waypointAngles[2] = newAngleThree;

    dataProcessing::updateWaypoints(worldPts);
    dataProcessing::updateWaypointsAngles(waypointAngles);

    if (dataProcessing::IsNearbyObject()) {
        // printf("CPCheck1\n");
        sdcVisibleObject* obj = dataProcessing::GetNearbyObject();
        // printf("CPCheck2\n");
        std::cout << "detectedYet? " << obj->getBrightnessDetected() << std::endl;
        if (!obj->getBrightnessDetected()){
              updateObjectBrightness(obj);
        }
    }
    // printf("CPCheck0\n");


    imshow("img", image);
    imwrite("waypoints.png", image);
    waitKey(4);
}

double CameraPlugin::getAngle(const cv::Point2d& p1, const cv::Point2d& p2)
{
  double angle = atan((p2.x - p1.x) / (p1.y - p2.y));

  if (p2.x - p1.x < 0 && p2.y - p1.y >= 0)
    angle = PI - angle;
  //rotates angle to quadrant 3
  else if (p2.x - p1.x < 0 && p2.y - p1.y < 0)
    angle += PI;
  //rotates angle to quadrant 4
  else if (p2.x - p1.x >= 0 && p2.y - p1.y < 0)
    angle = 2*PI - angle;
  return angle;
}

void CameraPlugin::ROI(Mat &mat, int lo, int hi)
{
    for(size_t i = 0; i < mat.rows; i++)
    {
        if(i < lo || i > hi)
        {
            for(size_t j = 0; j < mat.cols; j++)
            {
                mat.at<Vec3b>(i,j)[0] = 0;
                mat.at<Vec3b>(i,j)[1] = 0;
                mat.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }
}

std::pair<cv::Point2d, cv::Point> CameraPlugin::vanishPoint(Mat mat, int lo)
{
    int roi_ID = lo/130;
    int houghVotes = 105;
    if(roi_ID == 0)
        houghVotes = 52;

    vector<Vec2f> lines;

    HoughLines(mat, lines, 1, PI/180, houghVotes, 0, 0);

    // inner most lines
    float rho_left = FLT_MAX, theta_left = FLT_MAX;
    float rho_right = FLT_MIN, theta_right = FLT_MIN;

    for(size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        if((0.15 < theta && theta < 1.54) || (theta > 1.62 && theta < 3))
        {
            if(theta > PI/2)
            {
                if(theta-PI/2 > theta_right-PI/2)
                {
                    theta_right = theta;
                    rho_right = rho;
                }
            }
            if(theta < PI/2)
            {
                if(theta-PI/2 < theta_left-PI/2)
                {
                    theta_left = theta;
                    rho_left = rho;
                }
            }
        }
    }

    // convert polar to Cartesian
    double a1 = -(cos(theta_left)/sin(theta_left));
    double b1 = rho_left/sin(theta_left);

    double a2 = -(cos(theta_right)/sin(theta_right));
    double b2 = rho_right/sin(theta_right);

    // find intersection of two equations
    /*double x = (b1-b2)/(a2-a1);
    double y = a1 * x + b1;

    double x2 = mat.cols/2;
    double y2 = mat.rows;

    // y = mx + n, x = (y-n)/m
    double m = (y2-y)/(x2-x);
    double n = -m*x+y;

    int waypoint_x = (lo-n)/m;*/

    // find x, given y = mid
    int waypoint_x1 = (lo-b1)/a1;
    int waypoint_x2 = (lo-b2)/a2;

    cv::Point2d p1 = cv::Point2d(waypoint_x1, lo);
    cv::Point2d p2 = cv::Point2d(waypoint_x2, lo);
    sidePoints.push_back(p1);
    sidePoints.push_back(p2);

    int waypoint_x = (waypoint_x1+waypoint_x2)/2;

    if(waypoint_x < 5){
        waypoint_x = mat.cols/2;
    }

    // draw detected lines & waypoints
    for(size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        if((0.15 < theta && theta < 1.54) || (theta > 1.62 && theta < 3))
        {
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line(mat, pt1, pt2, Scalar(128,128,128), 1);
        }
    }
    circle(mat, p1, 2, Scalar(255,255,255), 3);
    circle(mat, p2, 2, Scalar(255,255,255), 3);
    circle(mat, cv::Point(waypoint_x,lo), 2, Scalar(255,255,255), 3);

    imshow(std::to_string(roi_ID), mat);



    math::Vector3 originCoord;
    math::Vector3 direction;
    this->parentSensor->GetCamera(0)->GetCameraToViewportRay(waypoint_x, lo, originCoord, direction);

    // cout << "viewportWidth originCoord is " << originCoord << endl;
    // cout << "viewportWidth direction is " << direction << endl;

    // get the realworld coordinates
    double prop = - double(originCoord[2])/direction[2];
    double newX = prop * direction[0] + originCoord[0];
    double newY = prop * direction[1] + originCoord[1];

    // cout << "realworld X is" << newX << endl;
    // cout << "realworld Y is " << newY << endl;

    std::ofstream roadPoints;
    roadPoints.open("roadPoints.csv", std::ios_base::app);
    roadPoints << newX << ", " << newY << std::endl;

    return std::make_pair(cv::Point2d(newX,newY), cv::Point(waypoint_x,lo));
    //return cv::Point2d(waypoint_x,mid);
}

// Gray -> Gaussian Blur -> Morph Open -> Edge
Mat CameraPlugin::preprocess(Mat mat)
{
    Mat gray, morph, canny;
    cvtColor(mat, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, cv::Size(5,5), 0, 0);

    Mat erosion(7, 7, CV_8U, Scalar(1));
    morphologyEx(gray, morph, MORPH_OPEN, erosion);

    Canny(gray, canny, 128, 255);

    return canny;
}

///////////// obstacle code ////////////////
void CameraPlugin::updateObjectBrightness(sdcVisibleObject* visibleObject) {
    std::vector<Point2f> points_in_roi;

    int rightIndex = 640-visibleObject->getLeftRayIndex();
    int leftIndex = 640-visibleObject->getRightRayIndex();

    //callibrate this based on final camera position/angle
    int height = 50;

    // hard code the boundaries
    for (int i = leftIndex; i < rightIndex; i = i + (rightIndex-leftIndex)/5) {
        cv::Point2f point_to_avg(i, height);
        points_in_roi.push_back(point_to_avg);
    }

    int avgThickness = 2;
    int avgLineType = 8;
    int avgPointThickness = -1;
    int avgPointRadius = 2;
    int sum = 0;

    //get average color of points
    for (int i = 0; i < points_in_roi.size(); i++) {
        Vec3b intensity = image.at<cv::Vec3b>(points_in_roi[i].x, height);
        sum = sum + intensity.val[0]+ intensity.val[1] + intensity.val[2];
        circle(image, points_in_roi[i], avgPointRadius, Scalar( 0, 0, 255 ), avgPointThickness, avgLineType);
    }

    circle(image, points_in_roi[points_in_roi.size() - 1], avgPointRadius, Scalar( 0, 0, 255 ), avgPointThickness, avgLineType);

//    int blue_avg = (blue_sum/3)%255;
//    int green_avg = (green_sum/3)%255;
//    int red_avg = (red_sum/3)%255;
    int brightness = sum/3/points_in_roi.size();

    Vec3f avg_color(brightness, brightness, brightness);
    //std::cout << "obstacle color: " << brightness << std::endl;

    //-- Show detected keypoints
    //imshow("Average sample locations", image);

    visibleObject->SetBrightness(brightness);
    visibleObject->setBrightnessDetected();
}
