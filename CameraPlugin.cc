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
    double ROI_lo = height/9;
    double ROI_hi = height/1.02;

    Mat rect_roi(image.size(), image.type());
    image.copyTo(rect_roi);
    ROI(rect_roi, ROI_lo, ROI_hi);


    // Create sub ROIs
    int n_sub = 3;
    double interval = (ROI_hi-ROI_lo)/n_sub;

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
        sub_lo = sub_hi;
    }

    imshow("sub0", subs[0]);
    imshow("sub1", subs[1]);
    imshow("sub2", subs[2]);
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
        int lo = ROI_lo + i * interval;
        int hi = lo + interval;

        std::pair<cv::Point2d, cv::Point> pts = vanishPoint(proc_subs[i], lo);
        worldPts.push_back(std::get<0>(pts));
        imagePts.push_back(std::get<1>(pts));

        circle(image, std::get<1>(pts), 2, Scalar(255,0,0), 3);
    }

    cv::Point originaPoint = cv::Point2d(320,400);
    imagePts.push_back(originaPoint);

    double previousAngle = 0;
    for(int i = 3; i > 0; i--)
    {
        double angle = getAngle(imagePts[i].x, imagePts[i].y, imagePts[i-1].x, imagePts[i-1].y, previousAngle);
        previousAngle += angle;
        // std::cout << "angle " << i <<" is " << angle << '\n';
        waypointAngles.push_back(angle);
    }

    dataProcessing::updateWaypoints(worldPts);
    dataProcessing::updateWaypointsAngles(waypointAngles);

    imshow("img", image);
    imwrite("waypoints.png", image);
    waitKey(4);

    for (sdcVisibleObject* obj : dataProcessing::GetNearbyObjects()) {
      updateObjectBrightness(obj);
    }
}

double CameraPlugin::getAngle(int firstX, int firstY, int secondX, int secondY,
                              double previousAngle)
{
  double tangValue = (double)(secondX - firstX) / (double)(firstY - secondY);
  double angle = atan(tangValue) - previousAngle;
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

std::pair<cv::Point2d, cv::Point> CameraPlugin::vanishPoint(Mat mat, int mid)
{
    vector<Vec2f> lines;
    HoughLines(mat, lines, 1, PI/180, 38, 0, 0);

    // inner most lines
    float rho_left = FLT_MAX, theta_left = FLT_MAX;
    float rho_right = FLT_MIN, theta_right = FLT_MIN;

    for(size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        if((0.1 < theta && theta < 1.5) || (theta > 1.62 && theta < 3.14))
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
    double x = (b1-b2)/(a2-a1);
    double y = a1 * x + b1;

    double x2 = mat.cols/2;
    double y2 = mat.rows;

    // y = mx + n, x = (y-n)/m
    double m = (y2-y)/(x2-x);
    double n = -m*x+y;

    // find x, given y = mid
    int waypoint_x = (mid-n)/m;

    math::Vector3 originCoord;
    math::Vector3 direction;
    this->parentSensor->GetCamera(0)->GetCameraToViewportRay(waypoint_x, mid, originCoord, direction);

    // cout << "viewportWidth originCoord is " << originCoord << endl;
    // cout << "viewportWidth direction is " << direction << endl;

    // get the realworld coordinates
    double prop = - double(originCoord[2])/direction[2];
    double newX = prop * direction[0] + originCoord[0];
    double newY = prop * direction[1] + originCoord[1];

    // cout << "realworld X is" << newX << endl;
    // cout << "realworld Y is " << newY << endl;

    return std::make_pair(cv::Point2d(newX,newY), cv::Point(waypoint_x,mid));
    //return cv::Point2d(waypoint_x,mid);
}

// Gray -> Gaussian Blur -> Morph Open -> Edge
Mat CameraPlugin::preprocess(Mat mat)
{
    Mat gray, morph, canny;
    cvtColor(mat, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, cv::Size(5,5), 0, 0);

    Mat erosion(5, 5, CV_8U, Scalar(1));
    morphologyEx(gray, morph, MORPH_OPEN, erosion);

    Canny(gray, canny, 128, 255);

    return canny;
}

///////////// obstacle code ////////////////
void CameraPlugin::updateObjectBrightness(sdcVisibleObject* visibleObject) {
    std::vector<Point2f> points_in_roi;
    // TODO: This is maybe incorrect, angle code needs debugging
    // Assuming this is in pixels, print to check what output is
    double left_edge = visibleObject->getLeftRay().GetLongitudinalDist();
    double right_edge = visibleObject->getRightRay().GetLongitudinalDist();

    //callibrate this based on final camera position/angle
    int height = (int)this->parentSensor->GetImageHeight(0)/2;

    for (int i = left_edge; i < right_edge; i = i + (right_edge/5)) {
        cv::Point2f point_to_avg(i, height/2);
        points_in_roi.push_back(point_to_avg);
    }

    int avgThickness = 2;
    int avgLineType = 8;
    int avgPointThickness = -1;
    int avgPointRadius = 6;
    float red_sum, green_sum, blue_sum = 0;

    //get average color of points
    for (int i = 0; i < points_in_roi.size() - 1; i++) {
        Vec3b intensity = image.at<cv::Vec3b>(points_in_roi[i].x, 200);
        blue_sum += intensity.val[0];
        green_sum += intensity.val[1];
        red_sum += intensity.val[2];
        circle(image, points_in_roi[i], avgPointRadius, Scalar( 0, 0, 255 ), avgPointThickness, avgLineType);
    }

    circle(image, points_in_roi[points_in_roi.size() - 1], avgPointRadius, Scalar( 0, 0, 255 ), avgPointThickness, avgLineType);

    float blue_avg = blue_sum/3;
    float green_avg = green_sum/3;
    float red_avg = red_sum/3;

    Vec3f avg_color(blue_avg, green_avg, red_avg);
    std::cout << "obstacle color: " << blue_avg << ", " << green_avg << ", " << red_sum << std::endl;

    //-- Show detected keypoints
    imshow("Average sample locations", image);

    visibleObject->SetBrightness((blue_avg + green_avg + red_avg)/3);
}
