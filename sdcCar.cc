/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


/*
 * Based on UtilityCart.cc written by Nate Koenig, sdcCar provides both
 * interaction with Gazebo's simulation environment as well as logic to
 * make it behave intelligently in a variety of situations. This is the main
 * class used in the Self-Driving Comps project.
 *
 * Physics parameters and Gazebo interfacing are based on UtilityCart.
 */

#include "sdcCar.hh"

#include <string>
#include <vector>
#include <exception>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

#include "globals.hh"

#include "sdcAngle.hh"
#include "sdcHLC.hh"
#include "sdcIntersection.hh"
#include "sdcSensorData.hh"
#include "sdcWaypoint.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3 * PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);

////////////////////
// HELPER METHODS //
////////////////////

/*
 * Updates the list of objects in front of the car with the given list of new objects
 */
void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject*> newObjects) {
  if (frontObjects_.size() == 0) {
    // The car wasn't tracking any objects, so just set the list equal to the new list
    frontObjects_ = newObjects;
    return;
  }

  std::vector<bool> isOldObjectMissing;
  std::vector<bool> isBrandNewObject;
  for (int i = 0; i < newObjects.size(); i++) {
    isBrandNewObject.push_back(true);
  }

  // Compare each old object to the new objects, and determine
  // which of them are getting updated, which are missing, as well
  // as if any of the passed in objects are brand new
  for (int i = 0; i < frontObjects_.size(); i++) {
    sdcVisibleObject* oldObj = frontObjects_[i];
    isOldObjectMissing.push_back(true);

    for (int j = 0; j < newObjects.size(); j++) {
      // Only match each new object to one old object
      if (!isBrandNewObject[j]) continue;
      sdcVisibleObject* newObj = newObjects[j];

      if (oldObj->IsSameObject(newObj)) {
        oldObj->Update(newObj);
        frontObjects_[i] = oldObj;
        isOldObjectMissing[i] = false;
        isBrandNewObject[j] = false;
        break;
      }
    }
  }

  // Delete objects that are missing
  for (int i = isOldObjectMissing.size() - 1; i >= 0; i--) {
    if (isOldObjectMissing[i]) {
      frontObjects_.erase(frontObjects_.begin() + i);
    }
  }

  // Add brand new objects
  for (int i = 0; i < newObjects.size(); i++) {
    if (isBrandNewObject[i]) {
      frontObjects_.push_back(newObjects[i]);
    }
  }
}

/*
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards() const {
  sdcAngle velAngle = GetDirection();
  sdcAngle carAngle = GetOrientation();
  return (carAngle - velAngle).IsFrontFacing();
}

/*
 * Gets the speed of the car
 */
double sdcCar::GetSpeed() const {
  return sqrt(pow(velocity_.x,2) + pow(velocity_.y,2));
}

/*
 * Returns the distance to the waypoint
 */
double sdcCar::GetDistance(math::Vector2d navWaypoint) const {
  double x_dist = navWaypoint_.x - x_;
  double y_dist = navWaypoint_.y - y_;
  return sqrt(pow(x_dist, 2) + pow(y_dist, 2));
}

/*
 * Gets the current direction the car is travelling
 */
sdcAngle sdcCar::GetDirection() const {
  math::Vector3 velocity = velocity_;
  return sdcAngle(atan2(velocity.y, velocity.x));
}

/*
 * Gets the current direction the car is travelling in NSEW
 */
void sdcCar::GetNSEW() {
  if ((yaw_ - WEST).WithinMargin(PI/4)) {
    currentDir_ = west;
  } else if ((yaw_ - SOUTH).WithinMargin(PI/4)) {
    currentDir_ = south;
  } else if ((yaw_ - EAST).WithinMargin(PI/4)) {
    currentDir_ = east;
  } else {
    currentDir_ = north;
  }
}

/*
 * Returns the maximum amount of time we have to be worried about when
 * determining what to avoid in the future. Changes based on velocity of the
 * car.
 */
double sdcCar::GetMaxSafeTime() const {
  return 1;
}

/*
 * Gets the direction the car is facing
 */
sdcAngle sdcCar::GetOrientation() const {
  return yaw_;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) const {
  math::Vector2d position = sdcSensorData::GetPosition();
  math::Vector2d targetVector = math::Vector2d(target.x - position.x, target.y - position.y);
  return sdcAngle(atan2(targetVector.y, targetVector.x));
}

/*
 * Returns true if there is an object ahead of the car that might collide with us if we
 * continue driving straight ahead
 */
bool sdcCar::ObjectDirectlyAhead() const {
  if (frontObjects_.size() == 0) return false;

  for (int i = 0; i < frontObjects_.size(); i++) {
    if (IsObjectDirectlyAhead(frontObjects_[i])) {
      return true;
    }
  }
  return false;
}

/*
 * Returns true if the given object is directly ahead of us, else false
 */
bool sdcCar::IsObjectDirectlyAhead(const sdcVisibleObject* obj) const {
  double leftDist = obj->Left().GetLateralDist();
  double rightDist = obj->Right().GetLateralDist();
  if (leftDist < 0 && rightDist > 0) return true;
  return fmin(fabs(leftDist), fabs(rightDist)) < FRONT_OBJECT_COLLISION_WIDTH / 2.;
}

/*
 * Returns true if there is an object on a potential collision course with our car
 */
bool sdcCar::ObjectOnCollisionCourse() const {
  if (frontObjects_.size() == 0) return false;

  for (int i = 0; i < frontObjects_.size(); i++) {
    if (IsObjectOnCollisionCourse(frontObjects_[i])) {
      return true;
    }
  }
  return false;
}

/*
 * Returns true if the given object is on a potential collision course with our car
 */
bool sdcCar::IsObjectOnCollisionCourse(const sdcVisibleObject* obj) const {
  return IsObjectTooFast(obj) || IsObjectTooFurious(obj);
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
bool sdcCar::IsObjectTooFast(const sdcVisibleObject* obj) const {
  math::Vector2d centerpoint = obj->GetCenterPoint();
  bool inLineToCollide = fabs(obj->LineIntercept()) < 1.5 || (fabs(centerpoint.x) < 1.5 && fabs(obj->GetEstimatedXSpeed()) < fabs(0.1 * obj->GetEstimatedYSpeed()));
  bool willHitSoon = obj->Dist() / obj->GetEstimatedSpeed() < 20;
  return inLineToCollide && willHitSoon;
}

/*
 * Returns true if the given object is very close to the car
 */
bool sdcCar::IsObjectTooFurious(const sdcVisibleObject* obj) const {
  math::Vector2d centerpoint = obj->GetCenterPoint();
  return (fabs(centerpoint.x) < FRONT_OBJECT_COLLISION_WIDTH / 2. && fabs(centerpoint.y) < 1.5);
}

///////////////////////////
// BEGIN CONTROL METHODS //
///////////////////////////

/*
 * Sets the rate of acceleration for the car. The rate is a scalar for the
 * force applied to accelerate the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetAccelRate(double rate) {
  accelRate_ = fmax(rate, 0.0);
}

/*
 * Sets the rate of braking for the car. The rate is a scalar for the
 * force applied to brake the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetBrakeRate(double rate) {
  brakeRate_ = fmax(rate, 0.0);
}

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(sdcAngle direction) {
  targetDirection_ = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a) {
  targetSteeringAmount_ = a;
}

/*
 * Sets the steering amount for the steering wheel
 */
void sdcCar::SetSteeringAmount(double a) {
  steeringAmount_ = a;
}

/*
 * Sets the target speed for the car, as well as resetting the brake
 * and accel rates to default. Methods wishing to change those parameters
 * should make sure to do so AFTER a call to this method
 */
void sdcCar::SetTargetSpeed(double s) {
  targetSpeed_ = fmax(fmin(s, maxCarSpeed_), 0);
  stopping_ = (targetSpeed_ == 0);
  SetAccelRate();
  SetBrakeRate();
}

/*
 * Sets the amount by which the car turns. A larger number makes the car turn
 * harder.
 */
void sdcCar::SetTurningLimit(double limit) {
  turningLimit_ = limit;
}

//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////

/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */
void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the model and chassis of the car for later access
  model_ = _model;
  chassis_ = model_->GetLink(_sdf->Get<std::string>("chassis"));

  // Get all the wheel joints
  joints_[0] = model_->GetJoint(_sdf->Get<std::string>("front_left"));
  joints_[1] = model_->GetJoint(_sdf->Get<std::string>("front_right"));
  joints_[2] = model_->GetJoint(_sdf->Get<std::string>("back_left"));
  joints_[3] = model_->GetJoint(_sdf->Get<std::string>("back_right"));

  // Pull some parameters that are defined in the sdf
  maxSpeed_ = _sdf->Get<double>("max_speed");
  aeroLoad_ = _sdf->Get<double>("aero_load");
  tireAngleRange_ = _sdf->Get<double>("tire_angle_range");
  frontPower_ = _sdf->Get<double>("front_power");
  rearPower_ = _sdf->Get<double>("rear_power");
  wheelRadius_ = _sdf->Get<double>("wheel_radius");

  // Tell Gazebo to call OnUpdate whenever the car needs an update
  connections_.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init() {
  // Compute the angle ratio between the steering wheel and the tires
  steeringRatio_ = STEERING_RANGE / tireAngleRange_;

  // During init, sensors aren't available so pull position and rotation information
  // straight from the car
  math::Pose pose = chassis_->GetWorldPose();
  yaw_ = sdcAngle(pose.rot.GetYaw());
  x_ = pose.pos.x;
  y_ = pose.pos.y;
  // hlc_->GenerateWaypoints();
}

/*
 * Called whenever Gazebo needs an update for this model
 */
void sdcCar::OnUpdate() {
  // Get the current velocity of the car
  velocity_ = chassis_->GetWorldLinearVel();
  // Get the cars current position
  math::Vector2d pose = sdcSensorData::GetPosition();
  x_ = pose.x;
  y_ = pose.y;
  // Get the cars current rotation
  yaw_ = sdcSensorData::GetYaw();

  // Check if the front lidars have been updated, and if they have update
  // the car's list
  if (frontLidarLastUpdate_ != sdcSensorData::GetLidarLastUpdate(FRONT)) {
    std::vector<sdcVisibleObject*> v = sdcSensorData::GetObjectsInFront();
    UpdateFrontObjects(v);
    frontLidarLastUpdate_ = sdcSensorData::GetLidarLastUpdate(FRONT);
  }

  // Call our Drive function, which is the brain for the car
  hlc_->Drive();
  hlc_->update();

  ////////////////////////////
  // GAZEBO PHYSICS METHODS //
  ////////////////////////////

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAmount_ / steeringRatio_;

  // Compute the rotational velocity of the wheels
  double jointVel = (gas_-brake_ * maxSpeed_) / wheelRadius_;

  // Set velocity and max force for each wheel
  joints_[0]->SetVelocityLimit(1, -jointVel);
  joints_[0]->SetForce(1, -(gas_ * accelRate_ + brake_ * brakeRate_) * frontPower_);

  joints_[1]->SetVelocityLimit(1, -jointVel);
  joints_[1]->SetForce(1, -(gas_ * accelRate_ + brake_ * brakeRate_) * frontPower_);

  joints_[2]->SetVelocityLimit(1, -jointVel);
  joints_[2]->SetForce(1, -(gas_ * accelRate_ + brake_ * brakeRate_) * rearPower_);

  joints_[3]->SetVelocityLimit(1, -jointVel);
  joints_[3]->SetForce(1, -(gas_ * accelRate_ + brake_ * brakeRate_) * rearPower_);

  // Set the front-left wheel angle
  joints_[0]->SetLowStop(0, wheelAngle);
  joints_[0]->SetHighStop(0, wheelAngle);
  joints_[0]->SetLowStop(0, wheelAngle);
  joints_[0]->SetHighStop(0, wheelAngle);

  // Set the front-right wheel angle
  joints_[1]->SetHighStop(0, wheelAngle);
  joints_[1]->SetLowStop(0, wheelAngle);
  joints_[1]->SetHighStop(0, wheelAngle);
  joints_[1]->SetLowStop(0, wheelAngle);

  //  aerodynamics
  chassis_->AddForce(math::Vector3(0, 0, aeroLoad_ * velocity_.GetSquaredLength()));

  // Sway bars
  math::Vector3 bodyPoint;
  math::Vector3 hingePoint;
  math::Vector3 axis;

  // Physics calculations
  for (int ix = 0; ix < 4; ++ix) {
    hingePoint = joints_[ix]->GetAnchor(0);
    bodyPoint = joints_[ix]->GetAnchor(1);

    axis = joints_[ix]->GetGlobalAxis(0).Round();
    double displacement = (bodyPoint - hingePoint).Dot(axis);

    float amt = displacement * swayForce_;
    if (displacement > 0) {
      if (amt > 15)
        amt = 15;

      math::Pose p = joints_[ix]->GetChild()->GetWorldPose();
      joints_[ix]->GetChild()->AddForce(axis * -amt);
      chassis_->AddForceAtWorldPosition(axis * amt, p.pos);

      p = joints_[ix^1]->GetChild()->GetWorldPose();
      joints_[ix^1]->GetChild()->AddForce(axis * amt);
      chassis_->AddForceAtWorldPosition(axis * -amt, p.pos);
    }
  }
}

/*
 * Constructor for the car. Sets several parameters to default values, some of
 * which will get overwritten in Load or Init and others that will be updated
 * when the car is updating
 */
sdcCar::sdcCar() {
  hlc_ = new sdcHLC(this);

  joints_.resize(4);

  // Physics variables
  aeroLoad_ = 0.1;
  swayForce_ = 10;

  maxSpeed_ = 10;
  frontPower_ = 50;
  rearPower_ = 50;
  wheelRadius_ = 0.3;
  steeringRatio_ = 1.0;
  tireAngleRange_ = 1.0;

  // Movement parameters
  gas_ = 0.0;
  brake_ = 0.0;
  accelRate_ = 1.0;
  brakeRate_ = 1.0;

  // Limits on the car's speed
  maxCarSpeed_ = 10;
  maxCarReverseSpeed_ = -10;

  // Set starting speed parameters
  targetSpeed_ = 1;

  // Set starting turning parameters
  steeringAmount_ = 0.0;
  targetSteeringAmount_ = 0.0;
  targetDirection_ = sdcAngle(0);
  turningLimit_ = 20.0;
  targetSpeed_ = 1;

  // Booleans for the car's actions
  turning_ = false;
  reversing_ = false;
  stopping_ = false;

  // Variables for parking
  targetParkingAngle_ = sdcAngle(0.0);
  parkingAngleSet_ = false;
  isFixingParking_ = false;
  parkingSpotSet_ = false;

  // Variables for waypoint driving
  waypointProgress_ = 0;

  // Variables for intersections
  stoppedAtSign_ = false;
  ignoreStopSignsCounter_ = 0;
  atIntersection_ = 0;

  // Variables for following
  isTrackingObject_ = false;
  stationaryCount_ = 0;

  // Variables for avoidance
  trackingNavWaypoint_ = false;
}

sdcCar::~sdcCar() {
  delete hlc_;
}
