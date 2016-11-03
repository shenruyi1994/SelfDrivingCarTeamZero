/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects) {
    if (this->frontObjects.size() == 0) {
        // The car wasn't tracking any objects, so just set the list equal to the new list
        this->frontObjects = newObjects;
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
    for (int i = 0; i < this->frontObjects.size(); i++) {
        sdcVisibleObject oldObj = this->frontObjects[i];
        isOldObjectMissing.push_back(true);

        for (int j = 0; j < newObjects.size(); j++) {
            // Only match each new object to one old object
            if (!isBrandNewObject[j]) continue;
            sdcVisibleObject newObj = newObjects[j];

            if (oldObj.IsSameObject(newObj)) {
                oldObj.Update(newObj);
                this->frontObjects[i] = oldObj;
                isOldObjectMissing[i] = false;
                isBrandNewObject[j] = false;
                break;
            }
        }
    }

    // Delete objects that are missing
    for (int i = isOldObjectMissing.size() - 1; i >= 0; i--) {
        if (isOldObjectMissing[i]) {
            this->frontObjects.erase(this->frontObjects.begin() + i);
        }
    }

    // Add brand new objects
    for (int i = 0; i < newObjects.size(); i++) {
        if (isBrandNewObject[i]) {
            this->frontObjects.push_back(newObjects[i]);
        }
    }
}

/*
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards() {
    sdcAngle velAngle = GetDirection();
    sdcAngle carAngle = this->GetOrientation();
    return (carAngle - velAngle).IsFrontFacing();
}

/*
 * Gets the speed of the car
 */
double sdcCar::GetSpeed() {
    return sqrt(pow(this->velocity.x,2) + pow(this->velocity.y,2));
}

/*
 * Returns the distance to the waypoint
 */
double sdcCar::GetDistance(math::Vector2d navWaypoint) {
    double x_dist = this->navWaypoint.x - this->x;
    double y_dist = this->navWaypoint.y - this->y;
    return sqrt(pow(x_dist, 2) + pow(y_dist, 2));
}

/*
 * Gets the current direction the car is travelling
 */
sdcAngle sdcCar::GetDirection() {
    math::Vector3 velocity = this->velocity;
    return sdcAngle(atan2(velocity.y, velocity.x));
}

/*
 * Gets the current direction the car is travelling in NSEW
 */
void sdcCar::GetNSEW() {
    if ((this->yaw - WEST).WithinMargin(PI/4)) {
        this->currentDir = west;
    } else if ((this->yaw - SOUTH).WithinMargin(PI/4)) {
        this->currentDir = south;
    } else if ((this->yaw - EAST).WithinMargin(PI/4)) {
        this->currentDir = east;
    } else {
        this->currentDir = north;
    }
}

/*
 * Gets the direction the car is facing
 */
sdcAngle sdcCar::GetOrientation() {
    return this->yaw;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) {
    math::Vector2d position = sdcSensorData::GetPosition();
    math::Vector2d targetVector = math::Vector2d(target.x - position.x, target.y - position.y);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
}

/*
 * Returns true if there is an object ahead of the car that might collide with us if we
 * continue driving straight ahead
 */
bool sdcCar::ObjectDirectlyAhead() {
    if (this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if (this->IsObjectDirectlyAhead(this->frontObjects[i])) {
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is directly ahead of us, else false
 */
bool sdcCar::IsObjectDirectlyAhead(sdcVisibleObject obj) {
    double leftDist = obj.left.GetLateralDist();
    double rightDist = obj.right.GetLateralDist();
    if (leftDist < 0 && rightDist > 0) return true;
    return fmin(fabs(leftDist), fabs(rightDist)) < FRONT_OBJECT_COLLISION_WIDTH / 2.;
}

/*
 * Returns true if there is an object on a potential collision course with our car
 */
bool sdcCar::ObjectOnCollisionCourse() {
    if (this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if (this->IsObjectOnCollisionCourse(this->frontObjects[i])) {
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is on a potential collision course with our car
 */
bool sdcCar::IsObjectOnCollisionCourse(sdcVisibleObject obj) {
    bool isTooFast = this->IsObjectTooFast(obj);
    bool isTooFurious = this->IsObjectTooFurious(obj);
    return isTooFast || isTooFurious;
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
bool sdcCar::IsObjectTooFast(sdcVisibleObject obj) {
    math::Vector2d centerpoint = obj.GetCenterPoint();
    bool inLineToCollide = (fabs(obj.lineIntercept) < 1.5 || (fabs(centerpoint.x) < 1.5 && fabs(obj.GetEstimatedXSpeed()) < fabs(0.1 * obj.GetEstimatedYSpeed())));
    bool willHitSoon = obj.dist / obj.GetEstimatedSpeed() < 20;
    return inLineToCollide && willHitSoon;
}

/*
 * Returns true if the given object is very close to the car
 */
bool sdcCar::IsObjectTooFurious(sdcVisibleObject obj) {
    math::Vector2d centerpoint = obj.GetCenterPoint();
    return (fabs(centerpoint.x) < FRONT_OBJECT_COLLISION_WIDTH / 2. && fabs(centerpoint.y) < 1.5);
}

///////////////////////////
// BEGIN CONTROL METHODS //
///////////////////////////

/*
 * Sets the rate of acceleration for the car. The rate is a scalar for the
 * force applies to accelerate the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetAccelRate(double rate) {
    this->accelRate = fmax(rate, 0.0);
}

/*
 * Sets the rate of braking for the car. The rate is a scalar for the
 * force applied to brake the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetBrakeRate(double rate) {
    this->brakeRate = fmax(rate, 0.0);
}

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(sdcAngle direction) {
    this->targetDirection = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a) {
    this->targetSteeringAmount = a;
}

/*
 * Sets the target speed for the car, as well as resetting the brake
 * and accel rates to default. Methods wishing to change those parameters
 * should make sure to do so AFTER a call to this method
 */
void sdcCar::SetTargetSpeed(double s) {
    this->targetSpeed = fmax(fmin(s, this->maxCarSpeed), 0);
    this->stopping = (this->targetSpeed == 0);
    this->SetAccelRate();
    this->SetBrakeRate();
}

/*
 * Sets the amount by which the car turns. A larger number makes the car turn
 * harder.
 */
void sdcCar::SetTurningLimit(double limit) {
    this->turningLimit = limit;
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
    this->model = _model;
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));

    // Get all the wheel joints
    this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
    this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
    this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
    this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));

    // Pull some parameters that are defined in the sdf
    this->maxSpeed = _sdf->Get<double>("max_speed");
    this->aeroLoad = _sdf->Get<double>("aero_load");
    this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");
    this->wheelRadius = _sdf->Get<double>("wheel_radius");

    // Tell Gazebo to call OnUpdate whenever the car needs an update
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init() {
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;

    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    // hlc_->GenerateWaypoints();
}

/*
 * Called whenever Gazebo needs an update for this model
 */
void sdcCar::OnUpdate() {
    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Vector2d pose = sdcSensorData::GetPosition();
    this->x = pose.x;
    this->y = pose.y;
    // Get the cars current rotation
    this->yaw = sdcSensorData::GetYaw();

    // Check if the front lidars have been updated, and if they have update
    // the car's list
    if (this->frontLidarLastUpdate != sdcSensorData::GetLidarLastUpdate(FRONT)) {
        std::vector<sdcVisibleObject> v = sdcSensorData::GetObjectsInFront();
        this->UpdateFrontObjects(v);
        this->frontLidarLastUpdate = sdcSensorData::GetLidarLastUpdate(FRONT);
    }

    // Call our Drive function, which is the brain for the car
    this->hlc_->Drive();


    ////////////////////////////
    // GAZEBO PHYSICS METHODS //
    ////////////////////////////

    // Compute the angle of the front wheels.
    double wheelAngle = this->steeringAmount / this->steeringRatio;

    // Compute the rotational velocity of the wheels
    double jointVel = (this->gas-this->brake * this->maxSpeed) / this->wheelRadius;

    // Set velocity and max force for each wheel
    this->joints[0]->SetVelocityLimit(1, -jointVel);
    this->joints[0]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[1]->SetVelocityLimit(1, -jointVel);
    this->joints[1]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[2]->SetVelocityLimit(1, -jointVel);
    this->joints[2]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    this->joints[3]->SetVelocityLimit(1, -jointVel);
    this->joints[3]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    // Set the front-left wheel angle
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);

    // Set the front-right wheel angle
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);

    //  aerodynamics
    this->chassis->AddForce(math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

    // Sway bars
    math::Vector3 bodyPoint;
    math::Vector3 hingePoint;
    math::Vector3 axis;

    // Physics calculations
    for (int ix = 0; ix < 4; ++ix) {
        hingePoint = this->joints[ix]->GetAnchor(0);
        bodyPoint = this->joints[ix]->GetAnchor(1);

        axis = this->joints[ix]->GetGlobalAxis(0).Round();
        double displacement = (bodyPoint - hingePoint).Dot(axis);

        float amt = displacement * this->swayForce;
        if (displacement > 0) {
            if (amt > 15)
                amt = 15;

            math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
            this->joints[ix]->GetChild()->AddForce(axis * -amt);
            this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

            p = this->joints[ix^1]->GetChild()->GetWorldPose();
            this->joints[ix^1]->GetChild()->AddForce(axis * amt);
            this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
        }
    }
}

/*
 * Constructor for the car. Sets several parameters to default values, some of
 * which will get overwritten in Load or Init and others that will be updated
 * when the car is updating
 */
sdcCar::sdcCar() {
    this->hlc_ = new sdcHLC(this);

    this->joints.resize(4);

    // Physics variables
    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    // Movement parameters
    this->gas = 0.0;
    this->brake = 0.0;
    this->accelRate = 1.0;
    this->brakeRate = 1.0;

    // Limits on the car's speed
    this->maxCarSpeed = 10;
    this->maxCarReverseSpeed = -10;

    // Set starting speed parameters
    this->targetSpeed = 6;

    // Set starting turning parameters
    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = sdcAngle(0);
    this->turningLimit = 20.0;

    // Booleans for the car's actions
    this->turning = false;
    this->reversing = false;
    this->stopping = false;

    // Variables for parking
    this->targetParkingAngle = sdcAngle(0.0);
    this->parkingAngleSet = false;
    this->isFixingParking = false;
    this->parkingSpotSet = false;

    // Variables for waypoint driving
    this->waypointProgress = 0;

    // Variables for intersections
    this->stoppedAtSign = false;
    this->ignoreStopSignsCounter = 0;
    this->atIntersection = 0;

    // Variables for following
    this->isTrackingObject = false;
    this->stationaryCount = 0;

    // Variables for avoidance
    this->trackingNavWaypoint = false;
}

sdcCar::~sdcCar() {
    delete hlc_;
}
