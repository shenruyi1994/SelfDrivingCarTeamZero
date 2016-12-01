#include "sdcLLC.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"
#include "sdcAngle.hh"
#include "dubins.hh"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace gazebo;


dubins::dubins(){
}

std::vector<int>  lsl(double initD, double finalD, double dist){
  // length of first left turn, p is # of timesteps for straight, q is # of timesteps for second lef\
  t turn;                                                                                              
  double t, p, q;

  t = -initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)));

  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));

  q = finalD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)));

  std::cout << "t is: " << t << "\n" << "p is: " <<  p << "\n" << "q is: " <<  q << "\n";

}


int dubins::calculateDubins(Waypoints* waypoints) {

  //direction in radians
  double initDirection = 0;
  cv::Point initPosition = cv::Point(0,0);

  //direction in radians
  double finalDirection = 0;
  cv::Point initPoint = cv::Point(0,10);
  double distance = 10;

  //double distance = sdcLLC::car_->sdcCar::GetDistance(finalPosition);

  
  
  std::vector<int> test = lsl(initDirection, finalDirection, distance);
  return 0;
}


