#include "sdcLLC.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "globals.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"

using namespace gazebo;

sdcLLC::sdcLLC(sdcCar* car): car_(car) {
    std::pair<double, double> dubins  = calculateDubins(NULL);
    car_->SetTargetSteeringAmount(dubins.first);
    car_->SetTargetSpeed(dubins.first);
}

std::pair<double, double> sdcLLC::calculateDubins(Waypoints* waypoints) {
    // car_->x_;
    // car_->y_;
    // car_->sdcAngle;
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
