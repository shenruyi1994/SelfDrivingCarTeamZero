//sdcLLC.hh containsCGAL typedefs and includes - may restructure location
#include "sdcLLC.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <CGAl/Arrangement_2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include "arr_print.h"
#include "dataProcessing.hh"
#include "globals.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"

using namespace gazebo;

void sdcLLC::update() {
  // std::pair<SteeringAngle, TimeStep> dubins  = calculateDubins(NULL);
  car_->SetTargetSteeringAmount(0);
  car_->SetTargetSpeed(10);
  std::vector<cv::Point> waypoints = dataProcessing::getWaypoints();
  sdcAngle angle;
  angle = car_->GetDirection();
  std::cout << angle << std::endl;

  //  calculateDubins(&waypoints);
  }

sdcLLC::sdcLLC(sdcCar* car): car_(car) {}

std::pair<SteeringAngle, TimeStep> sdcLLC::calculateDubins(Waypoints* waypoints) {

  // Circular_arc_2 arc = Circular_arc_2(Point_2(10,0), Point_2(5,5), Point_2(0, 0));
  //Circle_2 circle = Circle_2 (Point_2(10,10), Point_2(1,8), Point_2(9, 10));
  //Segment_2 seg1 = Segment_2(Point_2(0,0), Point_2(4,20));

  //Circle_2 LSCircle;
  //Circle_2 RSCircle;
  //Circle_2 LECircle;
  //Circle_2 RECIrcle;

  //Segment_2 LLSeg;
  //Segment_2 LRSeg;
  //Segment_2 RLSeg;
  // Segment_2 RRSeg;

  //Circular_arc_2 LSArc;
  //Circular_arc_2 RSArc;
  //Circular_arc_2 LEArc;
  //Circular_arc_2 REArc;

  //list holding our linesegments, circles, and segment arcs
  std::list<Curve> curves;

  //creates a circle centered at oridin with squaired raidus of 2

  Circle c1 = Circle(Rational_point(0,0), Number_type(2));
  curves.push_back(Curve(c1));

  //creates a line segment (x = y)
  Segment s1 = Segment(Rational_point(-2, -2), Rational_point(2, 2));

  curves.push_back(Curve(s1));

  //Creates circular arc defined by 3 non-colinear points
  Rational_point p1 = Rational_point(0,5);
  Rational_point p2 = Rational_point(3,4);
  Rational_point p3 = Rational_point(2,3);
  curves.push_back(Curve(p1, p2, p3));

  Arrangement arr;
  insert(arr, curves.begin(), curves.end());
  print_arrangement(arr);
}

/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Accelerate(double amt, double rate) {
  car_->SetTargetSpeed(car_->GetSpeed() + amt);
  car_->SetAccelRate(rate);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Brake(double amt, double rate) {
  car_->SetTargetSpeed(car_->GetSpeed() - amt);
  car_->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcLLC::Stop() {
  car_->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired,
 * NOT the direction the front of the car is facing
 */
void sdcLLC::Reverse() {
  car_->reversing_ = true;
}

/*
 * Stop reversing the car.
 */
void sdcLLC::StopReverse() {
  car_->reversing_ = false;
}
