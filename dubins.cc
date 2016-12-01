#include "sdcLLC.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"
#include "sdcAngle.hh"
#include "dubins.hh"
#include <opencv2/opencv.hpp>

using namespace gazebo;


dubins::dubins(){
}


Control dubins::calculateDubins(Waypoints* waypoints) {

  //sdcAngle initDirection = car_->GetDirection();
  cv::Point initPosition = cv::Point(0,0);

  sdcAngle finalDirection = 1;
  math::Vector2d finalPosition = math::Vector2d(120,120);
  //double distance = sdcLLC::car_->sdcCar::GetDistance(finalPosition);


  Control ret;
  return ret;
}
