#include "sdcLLC.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

//#include <CGAL/Exact_spherical_kernel_3.h>
//#include <CGAL/Random.h>


#include "globals.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"

using namespace gazebo;

//typedef CGAL::Exact_spherical_kernel_3         Spherical_k;
//typedef CGAL::Point_3<Spherical_k>             Point_3;
//typedef CGAL:: Sphere_3<Spherical_k>           Sphere_3;
sdcLLC::sdcLLC(sdcCar* car): car_(car) {
    std::pair<double, double> dubins  = calculateDubins(NULL);
    this->car_->SetTargetSteeringAmount(dubins.first);
    this->car_->SetTargetSpeed(dubins.first);
}

std::pair<double, double> sdcLLC::calculateDubins(Waypoints* waypoints) {
    // this->car_->x;
    // this->car_->y;
    // this->car_->sdcAngle;
    double velocity;
    double yaw;

    return std::make_pair(velocity, yaw);
}

/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Accelerate(double amt, double rate) {
    this->car_->SetTargetSpeed(this->car_->GetSpeed() + amt);
    this->car_->SetAccelRate(rate);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcLLC::Brake(double amt, double rate) {
    this->car_->SetTargetSpeed(this->car_->GetSpeed() - amt);
    this->car_->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcLLC::Stop() {
    this->car_->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired,
 * NOT the direction the front of the car is facing
 */
void sdcLLC::Reverse() {
    this->car_->reversing = true;
}

/*
 * Stop reversing the car.
 */
void sdcLLC::StopReverse() {
    this->car_->reversing = false;
}
