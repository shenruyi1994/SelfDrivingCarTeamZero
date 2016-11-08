#include "sdcLLC.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"


#include "globals.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"

#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Segment_2.h>


using namespace gazebo;

typedef CGAL::Exact_circular_kernel_2             Circular_k;
typedef CGAL::Point_2<Circular_k>                 Point_2;
typedef CGAL::Circle_2<Circular_k>                Circle_2;
typedef CGAL::Segment_2<Circular_k>               Segment_2; 
typedef CGAL::Circular_arc_2<Circular_k>          Circular_arc_2;



void sdcLLC::update() {
  std::pair<SteeringAngle, TimeStep> dubins  = calculateDubins(NULL);
  car_->SetTargetSteeringAmount(0);
  car_->SetTargetSpeed(10);
  std::cout << "Can we output to gazebo?" << std::endl;

  }

sdcLLC::sdcLLC(sdcCar* car): car_(car) {

}

std::pair<SteeringAngle, TimeStep> sdcLLC::calculateDubins(Waypoints* waypoints) {

  Circular_arc_2 arc = Circular_arc_2(Point_2(10,0), Point_2(5,5), Point_2(0, 0));
  Circle_2 circle = Circle_2 (Point_2(10,10), Point_2(1,8), Point_2(9, 10));
  Segment_2 seg1 = Segment_2(Point_2(0,0), Point_2(4,20));

  Circle_2 LSCircle;
  Circle_2 RSCircle;
  Circle_2 LECircle;
  Circle_2 RECIrcle;

  Segment_2 LLSeg;
  Segment_2 LRSeg;
  Segment_2 RLSeg;
  Segment_2 RRSeg;

  Circular_arc_2 LSArc;
  Circular_arc_2 RSArc;
  Circular_arc_2 LEArc;
  circular_arc_2 REArc;
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
