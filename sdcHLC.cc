#include "sdcHLC.hh"

#include <math.h>
#include <vector>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "globals.hh"
#include "dataProcessing.hh"
#include "sdcBoundingBox.hh"
#include "sdcBoundingCircle.hh"
#include "sdcCar.hh"
#include "sdcIntersection.hh"
#include "sdcUtils.hh"
#include "sdcLLC.hh"
#include "sdcRotatedBoundingBox.hh"
#include "Waypoints.hh"



using namespace gazebo;

std::vector<sdcWaypoint> WAYPOINT_VEC;

sdcHLC::sdcHLC(sdcCar* car): car_(car) {
  llc_ = new sdcLLC(car_);

  // Initialize state enums
  DEFAULT_STATE = WAYPOINT;
  currentState_ = DEFAULT_STATE;

  currentPerpendicularState_ = backPark;
  currentParallelState_ = rightBack;
  currentAvoidanceState_ = notAvoiding;

  lastUpdateTime_ = common::Time(0);

  lastX_ = car_->x_;
  lastY_ = car_->y_;
}

sdcHLC::~sdcHLC() {
  delete llc_;
}

////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

/*
 * Handles all logic for driving, is called every time the car receives an update
 * request from Gazebo
 */
void sdcHLC::Drive() {
  dataProcessing::UpdateCarDirection();
  common::Time curTime = car_->model_->GetWorld()->GetSimTime();
  if (curTime.Double() - lastUpdateTime_.Double() < .01) {
    UpdatePathDistance();
    return;
  } else {
    lastUpdateTime_ = common::Time(curTime);
  }

  if (dataProcessing::IsNearbyObject()) {
    Avoidance();
  }
  
  // Obstacle not detected -> keep following waypoints
  FollowWaypoints();

  // Attempts to turn towards the target direction
  MatchTargetDirection();

  // Attempts to match the target speed
  MatchTargetSpeed();
  
  /*
  // If not in avoidance, check if we should start following the thing
  // in front of us. If following is done, kick out to default state
  if (currentState_ != INTERSECTION && currentState_ != AVOIDANCE) {
    // If there's a stop sign, assume we're at an intersection
    if (car_->ignoreStopSignsCounter_ == 0 && sdcSensorData::stopSignFrameCount > 5) {
      currentState_ = INTERSECTION;
    }

    // If something is ahead of us, default to trying to follow it
    if (car_->ObjectDirectlyAhead()) {
      currentState_ = FOLLOW;
    } else if (currentState_ == FOLLOW && !car_->isTrackingObject_) {
      currentState_ = DEFAULT_STATE;
    }

    // Look for objects in danger of colliding with us, react appropriately
    if (car_->ObjectOnCollisionCourse()) {
      currentState_ = AVOIDANCE;
    }
  }

  car_->ignoreStopSignsCounter_ = fmax(car_->ignoreStopSignsCounter_ - 1, 0);


  // Possible states: stop, waypoint, intersection, follow, avoidance
  switch(currentState_) {
    // Final state, car is finished driving
    case STOP:
      llc_->Stop();
      break;

    // Default state; drive straight to target location
    case  WAYPOINT:
      // Handle lane driving

      llc_->Accelerate();
      // llc_->Stop();
      //WaypointDriving(WAYPOINT_VEC);
      break;

    // At a stop sign, performing a turn
    case INTERSECTION:
      if (car_->stoppedAtSign_ && car_->stationaryCount_ > 2000) {
        currentState_ = DEFAULT_STATE;
        car_->ignoreStopSignsCounter_ = 3000;
      } else if (car_->stoppedAtSign_ && car_->GetSpeed() < 0.5) {
        car_->stationaryCount_++;
      } else if (!car_->stoppedAtSign_ && sdcSensorData::sizeOfStopSign > 6000) {
        llc_->Stop();
        car_->stoppedAtSign_ = true;
        car_->stationaryCount_ = 0;
      }

    break;

    // Follows object that is going in same direction/towards same target
    case FOLLOW:
      Follow();
      // Handle lane driving
      break;

    // Smarter way to avoid objects; stopping, swerving, etc.
    case AVOIDANCE:
      // Cases: stop, swerve, go around
      Avoidance();
      break;

    // Parks the car
    case PARKING:
      PerpendicularPark();
      // ParallelPark();
      break;
  } */
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcHLC::MatchTargetDirection() {
  dataProcessing::UpdateCarDirection();
  sdcAngle directionAngleChange = car_->GetDirection() - car_->targetDirection_;
  // If the car needs to turn, set the target steering amount
  if (!directionAngleChange.WithinMargin(DIRECTION_MARGIN_OF_ERROR)) {
    // The steering amount scales based on how far we have to turn, with upper and lower limits
    double proposedSteeringAmount =
      fmax(fmin(-car_->turningLimit_ * tan(directionAngleChange.angle/-2),
            car_->turningLimit_), -car_->turningLimit_);

    // When reversing, steering directions are inverted
    if (!car_->reversing_) {
      car_->SetTargetSteeringAmount(proposedSteeringAmount);
    } else {
      car_->SetTargetSteeringAmount(-proposedSteeringAmount);
    }
  }

  // Check if the car needs to steer, and apply a small turn in the corresponding direction
  if (!(std::abs(car_->targetSteeringAmount_ - car_->steeringAmount_) < STEERING_MARGIN_OF_ERROR)) {
    if (car_->steeringAmount_ < car_->targetSteeringAmount_) {
      car_->steeringAmount_ =
        car_->steeringAmount_ + STEERING_ADJUSTMENT_RATE;
    } else {
      car_->steeringAmount_ =
        car_->steeringAmount_ - STEERING_ADJUSTMENT_RATE;
    }
  }
}

/*
 * Attempts to match the current target speed
 */
void sdcHLC::MatchTargetSpeed() {
  // Invert all the values if the car should be moving backwards
  int dirConst = car_->reversing_ ? -1 : 1;

  // If the car is moving the wrong direction or slower than the target speed, press on the gas
  if ((car_->reversing_ && car_->IsMovingForwards())
      || (!car_->reversing_ && !car_->IsMovingForwards())
      || (car_->GetSpeed() < car_->targetSpeed_)) {
    car_->gas_ = 1.0 * dirConst;
    car_->brake_ = 0.0;
  } else if (car_->GetSpeed() > car_->targetSpeed_) {
    // If the car is moving faster than the target speed, brake to slow down
    car_->gas_ = 0.0;
    if (car_->reversing_ != car_->IsMovingForwards()) {
      car_->brake_ = -2.0 * dirConst;
    } else {
      // If the car is drifting in the opposite direction it should be, don't brake
      // as this has the side effect of accelerating the car in the opposite direction
      car_->brake_ = 0.0;
    }
  }
}

/*
 * Drive from point to point in the given list
 */
void sdcHLC::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
  int progress = car_->waypointProgress_;
  if (progress < WAYPOINT_VEC.size()) {
    // Pull the next waypoint and set the car to drive towards it


    llc_->Accelerate();

    // Check if the car is close enough to the target to move on
    double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - car_->x_,2) + pow(WAYPOINT_VEC[progress].pos.second - car_->y_,2));
    if (distance < 7) {
      car_->turning_ = true;
    }
    if (car_->turning_ == true) {
      car_->SetTurningLimit(20);
      GridTurning(WAYPOINT_VEC[progress].waypointType);
    } else {
      math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
      sdcAngle targetAngle = car_->AngleToTarget(nextTarget);
      car_->SetTargetDirection(targetAngle);
      // LanedDriving();
    }
  } else if (car_->isFixingParking_) {
    car_->isFixingParking_ = false;
    currentState_ = PARKING;
    currentPerpendicularState_ = straightPark;
  } else {
    currentState_ = STOP;
  }
}

/*
 * Drive along a road or other surface, following waypoints. Rather than
 * directly following a path, aim for a point in front of the vehicle. This
 * less accurately follows the waypoints but provides a smoother transition
 * along sharp turns in the curve.
 */
void sdcHLC::FollowWaypoints() {
  car_->SetTargetSpeed(3);

  cv::Point2d targetPoint = FindDubinsTargetPoint();
  //printf("targetPoint: (%f, %f)\n", targetPoint.x, targetPoint.y);
  //printf("  speed: %f\n", car_->GetSpeed());
  //printf("  location: (%f, %f)\n", car_->x_, car_->y_);
  // AngleWheelsTowardsTarget(to_math_vec(targetPoint));
  car_->SetTargetPoint(targetPoint);
}

/*
 * Calculate the turning angle necessary to reach the provided point, and then
 * set the wheel angle correctly. Based on the tuned pure pursuit algorithm
 * found in this paper:
 * http://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
 */
void sdcHLC::AngleWheelsTowardsTarget(const math::Vector2d& target) {
  sdcAngle alpha = car_->AngleToTarget(target);
  double numerator = 2 * WHEEL_BASE * sin(alpha.angle);
  double denominator = lookaheadScalor_ * car_->GetSpeed();

  sdcAngle directionAngle = atan2(numerator, denominator);

  // We can't set the wheel angle directly, so instead we set the steering
  //printf("    +++++ directionAngle: %f\n", directionAngle.angle);
  //printf("    +++++ steeringAmount: %f\n", directionAngle.angle * car_->steeringRatio_);
  // car_->SetSteeringAmount(directionAngle.angle * car_->steeringRatio_);
}

/*
 * Updates the distance the car has travelled along the current dubins path
 */
void sdcHLC::UpdatePathDistance() {
  // TODO: implement small randomization of location to make it more realistic,
  //       rather than using the exact GPS coordinates

  // common::Time dt = car_->model_->GetWorld()->GetSimTime() - lastTime_;
  if (car_->model_->GetWorld()->GetSimTime().Double() < 0.01) {
    pathDist_ = 0;
  }
  // double avgVelocity = (car_->GetSpeed() + lastSpeed_) / 2;
  // lastTime_ = car_->model_->GetWorld()->GetSimTime();
  // lastSpeed_ = car_->GetSpeed();

  // double distanceTravelled = avgVelocity * dt.Double();
  pathDist_ += pythag_thm(car_->x_ - lastX_, car_->y_ - lastY_);
  lastX_ = car_->x_;
  lastY_ = car_->y_;
}

/*
 * Returns the point along the dubins path that the car should be following.
 */
cv::Point2d sdcHLC::FindDubinsTargetPoint() {
  cv::Point2d location = cv::Point2d(car_->x_, car_->y_);
  //printf("pathdist_: %f\n", pathDist_);
  // double lookaheadDistance = ScaledLookaheadDistance();
  double lookaheadDistance = 20;

  if (llc_->BeyondPath(pathDist_ + lookaheadDistance)) {
    llc_->GenerateNewDubins();
    pathDist_ = 0;
  }
  cv::Point2d tempTarget = llc_->GetDubinsPoint(lookaheadDistance);

  // double distanceToDubins = coord_distance(location, tempTarget);
  // double adjustment = distanceToDubins - lookaheadDistance;
  // double maxError = 0.1;
  // double tempPathDist = pathDist_;
  //
  // // finds a point along the dubins path that is beyond the ideal target distance
  // while (distanceToDubins < lookaheadDistance) {
  //   tempPathDist += adjustment;
  //   distanceToDubins = coord_distance(location, llc_->GetDubinsPoint(tempPathDist));
  //   adjustment *= 2;
  // }
  //
  // // does a binary search to find the correct distance along the path that we
  // // need to aim for
  // while (fabs(distanceToDubins - lookaheadDistance) > maxError) {
  //   if (distanceToDubins > lookaheadDistance) {
  //     tempPathDist -= adjustment;
  //   } else {
  //     tempPathDist += adjustment;
  //   }
  //   distanceToDubins = coord_distance(location, llc_->GetDubinsPoint(tempPathDist));
  //   adjustment *= .5;
  // }

  return tempTarget;
}

/*
 * Returns the lookahead distance scaled to account for linear velocity of the
 * vehicle. Only returns values within the range [lookaheadMin_, lookaheadMax_].
 */
double sdcHLC::ScaledLookaheadDistance() const {
  double tempDist = lookaheadScalor_ * car_->GetSpeed();
  return fmin(fmax(tempDist, lookaheadMin_), lookaheadMax_);
}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcHLC::LanedDriving() {
  int lanePos = sdcSensorData::LanePosition();
  car_->SetTurningLimit(sdcSensorData::GetNewSteeringMagnitude());
  if (!(lanePos > 320 || lanePos < -320)) {
    // It's beautiful don't question it
    sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
    car_->SetTargetDirection(car_->GetDirection() + laneWeight);
  }
}

////////////////////////////
////////////////////////////
// END WAYPOINT FOLLOWING //
////////////////////////////
////////////////////////////

/*
 * Car follows an object directly in front of it and slows down to stop when it starts to get close
 */
void sdcHLC::Follow() {
  // There's nothing in front of the car, so break out of follow
  if (car_->frontObjects_.size() == 0) {
    car_->isTrackingObject_ = false;
    currentState_ = DEFAULT_STATE;
    return;
  }

  // The default object to follow is directly in front of the car, the max range away

  // Kirsten + Ruyi : QUESTION: we are not using sdcSensorData but dataProcessing to pass lidar data right?
  sdcVisibleObject* tracked = new sdcVisibleObject(
    sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)),
    sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)),
    sdcSensorData::GetLidarMaxRange(FRONT),0,0);

  // Already tracking an object, find it again
  if (car_->isTrackingObject_) {
    bool foundTrackedObject = false;
    for (int i = 0; i < car_->frontObjects_.size(); i++) {
      sdcVisibleObject* obj = car_->frontObjects_[i];
      if (obj->IsTracking()) {
        tracked = obj;
        foundTrackedObject = true;
        break;
      }
    }
    if (!foundTrackedObject) {
      car_->isTrackingObject_ = false;
      return;
    }
  } else {
    // Not tracking an object, find one that's in front of the car
    // and start tracking it
    for (int i = 0; i < car_->frontObjects_.size(); i++) {
      sdcVisibleObject* obj = car_->frontObjects_[i];
      if (car_->IsObjectDirectlyAhead(obj)) {
        tracked = obj;
        tracked->SetTracking(true);
        car_->isTrackingObject_ = true;
        car_->frontObjects_[i] = tracked;
        break;
      }
    }
  }

  // After the above loops, if not following anything just return
  if (!car_->isTrackingObject_) return;

  math::Vector2d objCenter = tracked->GetCenterPoint();
  double objSpeed = tracked->GetEstimatedYSpeed();

  // Scale our speed based on how far away the tracked object is
  // The equation is 'scaledSpeed = (objY - 10)^3 / 2000.' which
  // gives a scaled speed of 0 at y=10 and +-0.5 at y=20, y=0 respectively
  double scaledSpeed = pow(objCenter.y - 10, 3) / 2000.;

  // Adjust the target speed based on the speed of the object, our speed,
  // and the above calculated scaled speed
  double newTargetSpeed = objSpeed + car_->GetSpeed() + scaledSpeed;
  car_->SetTargetSpeed(newTargetSpeed);

  // If the new target speed is sufficiently low, count the car as stationary
  if (newTargetSpeed < 0.3) {
    car_->stationaryCount_++;
  } else {
    car_->stationaryCount_ = 0;
  }

  // If the car has been stationary for sufficiently long, stop following and start
  // trying to navigate around the object in front of it
  if (car_->stationaryCount_ > 2000) {
    currentState_ = AVOIDANCE;
    currentAvoidanceState_ = navigation;
  }

  // Set the direction of the car to be angled at the tracked object
  if (objCenter.x != 0) {
    car_->SetTargetDirection(
      car_->GetOrientation()
      - sdcAngle(PI / 2.)
      + sdcAngle(atan2(objCenter.y, objCenter.x)));
  } else {
    car_->SetTargetDirection(car_->GetOrientation());
  }
}

/*
 * In avoidance, the car's only concern is not hitting objects. Provides a couple emergency methods, one
 * for stopping and one for swerving. Also provides a navigation case for maneuvering around objects in front
 * of the car
 */
void sdcHLC::Avoidance() {
  std::pair<cv::Point2d, cv::Point2d> obstacle = dataProcessing::getObstacleCoords();

  //Waypoint avoidPoint = Waypoint(obstacle.second.x, obstacle.second.y, car_->GetDirection());
  /*Waypoint avoidPoint;
  Waypoint carPoint;
  carPoint.x = car_->x_;
  carPoint.y = car_->y_;
  carPoint.direction = car_->GetDirection().angle;

  llc_->dubins_->calculateDubins(avoidPoint, carPoint, MIN_TURNING_RADIUS);

  steeringAngles.push_back(car_->GetDirection().angle);
  */
}

//////////////////////////////////////////
//////////////////////////////////////////
//  BEGIN COLLISION DETECTION FUNCTIONS //
//////////////////////////////////////////
//////////////////////////////////////////

void sdcHLC::DecideAvoidanceStrategy(const sdcVisibleObject* obj) {
  bool isCar = dataProcessing::GetObjectType(obj) == CAR_TYPE;

  if (roadState_ == STOP_16 && car_->GetSpeed() < 0.1) {
    roadState_ = WAIT_16;
  }

  // TODO: if the speed difference is small, then we should approach rather
  // than stop. This is only relevant if we also deal with moving obstacles.
  if (isCar && CanStopBeforeObject(obj)) {
    roadState_ = STOP_16;
  } else {
    roadState_ = AVOID_16;
  }
}

/*
 * Returns true if the car is able to stop before hitting the object.
 * Uses the equation d = v^2 / 20
 */
bool sdcHLC::CanStopBeforeObject(const sdcVisibleObject* obj) const {
  return obj->Dist() > sqrt(car_->GetSpeed()) / 20;
}

/*
 * Sets dangerousObj_ to the first object encountered that is on a collision
 * course with the car, or NULL if no such object exists.
 */
void sdcHLC::CheckNearbyObjectsForCollision() {
  for (sdcVisibleObject* obj : car_->frontObjects_) {
    if (IsObjectOnCollisionCourse(obj)) {
      dangerousObj_ = obj;
      return;
    }
  }
  dangerousObj_ = NULL;
}

/*
 * Checks if the object is on a collision course with the car.
 */
bool sdcHLC::IsObjectOnCollisionCourse(const sdcVisibleObject* obj) const {
  if (DoMaximumBoundingBoxesCollide(obj)) {
    double possibleCollisionTime = DoMaximumRadiiCollide(obj);
    return possibleCollisionTime != -1
      && DoAccurateVehicleShapesCollide(obj, possibleCollisionTime);
  }
  return false;
}

/*
 * Checks if the most pessimistic bounding boxes collide; that
 */
bool sdcHLC::DoMaximumBoundingBoxesCollide(const sdcVisibleObject* obj) const {
  double maxTime = car_->GetMaxSafeTime();
  math::Vector2d futureCarPos = GetPositionAtTime(maxTime);
  math::Vector2d futureObjPos = obj->GetProjectedPositionAtTime(maxTime);

  sdcBoundingBox carRect = sdcBoundingBox(
    fmin(car_->x_, futureCarPos.x),
    fmin(car_->y_, futureCarPos.y),
    fmax(car_->x_, futureCarPos.x),
    fmax(car_->y_, futureCarPos.y));
  sdcBoundingBox objRect = sdcBoundingBox(
    fmin(obj->GetCenterPoint().x, futureObjPos.x),
    fmin(obj->GetCenterPoint().y, futureObjPos.y),
    fmax(obj->GetCenterPoint().x, futureObjPos.x),
    fmax(obj->GetCenterPoint().y, futureObjPos.y));

  return carRect.DoesIntersect(objRect);
}

/*
 * Returns true if the distance between the car and the dangerous object is
 * ever within (max_radius_car + max_radius_obj) along their projected paths.
 */
double sdcHLC::DoMaximumRadiiCollide(const sdcVisibleObject* obj) const {
  for (int i = 0; i < car_->GetMaxSafeTime() * 100; i++) {
    if (DoMaximumRadiiCollideAtTime(obj, ((double)i) / 100)) {
      return ((double)i) / 100;
    }
  }
  return -1;
}

/*
 * Returns true if the distance between the car and obj at is within
 * (max_radius_car + max_radius_obj) at the given time
 */
bool sdcHLC::DoMaximumRadiiCollideAtTime(const sdcVisibleObject* obj,
                                         double time) const {
  sdcBoundingCircle selfCircle = sdcBoundingCircle(
    to_point(GetPositionAtTime(time)),
    pythag_thm(car_->width_, car_->length_)
  );

  // TODO: figure out how to estimate size of an object
  sdcBoundingCircle objCircle = sdcBoundingCircle(
    to_point(obj->GetProjectedPositionAtTime(time)),
    1 // placeholder for above TODO
  );

  return selfCircle.DoesIntersect(objCircle);
}

/*
 * Returns true if accurate shape depictions of the car and the object
 * ever intersect along their projected paths.
 * TODO: test the numbers (10, 100) to see if they are reasonable at all
 */
bool sdcHLC::DoAccurateVehicleShapesCollide(const sdcVisibleObject* obj,
                                            double possibleCollisionTime) const {
  for (int i = possibleCollisionTime * 100; i < (possibleCollisionTime + 10) * 100; i++) {
    if (DoAccurateVehicleShapesCollideAtTime(obj, ((double)i) / 100)) {
      return true;
    }
  }
  return false;
}

/*
 * Returns true if accurate shape depictions of the car and the object
 * intersect at the given time
 */
bool sdcHLC::DoAccurateVehicleShapesCollideAtTime(const sdcVisibleObject* obj,
                                                  double time) const {
  math::Vector2d selfPos = GetPositionAtTime(time);
  sdcRotatedBoundingBox selfBox = sdcRotatedBoundingBox(
    selfPos.x - car_->width_ / 2, selfPos.y + car_->width_ / 2,
    car_->width_, car_->width_,
    GetAngleAtTime(time).angle
  );

  // TODO: figure out a way to estimate the shape of an sdcVisibleObject
  return false;
}

/*
 * TODO: figure out how this will work. For now it is a placeholder, but will
 * eventually be based on the dubins path created by the LLC.
 */
math::Vector2d sdcHLC::GetPositionAtTime(double time) const {
  return math::Vector2d(0, 0);
}

/*
 * TODO: figure out how this will work. For now it is a placeholder, but will
 * eventually be based on the dubins path created by the LLC.
 */
sdcAngle sdcHLC::GetAngleAtTime(double time) const {
  return sdcAngle(0);
}

/*
 * Generates a new path for the car, given an object and a collision location
 * on a previous path
 * TODO: integrate this with the dubins path algorithms
 */
std::vector<sdcWaypoint*>* sdcHLC::ComputeAvoidancePath(
    sdcVisibleObject* obj, math::Vector2d collision) {

    return NULL;
}

/*
 * Returns the angle between the car and the vehicle at the time of collision,
 * allowing for improved avoidance decisionmaking.
 */
sdcAngle sdcHLC::GetCollisionAngleAtTime(const sdcVisibleObject* obj,
                                         double time) const {
  return sdcAngle();
}

///////////////////////////////////////////////
///////////////////////////////////////////////
// BEGIN OTHER STATEFUL ALGORITHMS FROM 2015 //
///////////////////////////////////////////////
///////////////////////////////////////////////

/*
 * Executes a turn at an intersection
 */
void sdcHLC::GridTurning(int turn) {
  int progress = car_->waypointProgress_;
  if (turn == 3) {
    car_->waypointProgress_++;
    currentState_ = STOP;
    return;
  } else if (turn == 0) {
    car_->waypointProgress_++;
    car_->turning_ = false;
    return;
  }
  math::Vector2d nextTarget = {
    WAYPOINT_VEC[progress+1].pos.first,
    WAYPOINT_VEC[progress+1].pos.second
  };
  sdcAngle targetAngle = car_->AngleToTarget(nextTarget);
  car_->SetTargetDirection(targetAngle);
  sdcAngle margin = car_->GetOrientation().FindMargin(targetAngle);
  if (margin < .1 && margin > -.1) {
    car_->turning_ = false;
    car_->waypointProgress_++;
  }
}


/*
 * Perpendicular back parking algorithm
 */
void sdcHLC::PerpendicularPark() {
  // Get back and side lidar rays that detect whether the back left and right bumpers of the car
  // will collide with other objects
  std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
  std::vector<double> rightBackSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_BACK);
  std::vector<double> leftBackSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_BACK);
  std::vector<double> rightFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_FRONT);
  std::vector<double> leftFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_FRONT);
  std::vector<double> backRightBound;
  std::vector<double> backMidBound;
  std::vector<double> backLeftBound;
  std::vector<double> backRightSideBound;
  std::vector<double> backLeftSideBound;
  bool isSafe = true;
  std::vector<sdcWaypoint> fix;
  math::Vector2d pos = sdcSensorData::GetPosition();
  car_->SetTurningLimit(30.0);

  // Store vector of rays from the back lidar that detect objects behind the car, specifically the middle
  // and far right and left rays
  int numBackRightRays = sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK);
  int rightSideBoundRange = numBackRightRays / 16;
  int midBackRightSideRay = numBackRightRays / 2;
  if (rightBackSideLidar.size() != 0) {
    for (int i = 0; i < rightSideBoundRange; i++) {
      backRightSideBound.push_back(rightBackSideLidar[i]);
    }
    for (int j = midBackRightSideRay - rightSideBoundRange / 2; j < midBackRightSideRay + rightSideBoundRange / 2; j++) {
      backRightSideBound.push_back(rightBackSideLidar[j]);
    }
    for (int k = numBackRightRays - rightSideBoundRange; k < numBackRightRays; k++) {
      backRightSideBound.push_back(rightBackSideLidar[k]);
    }
  }

  // Store vector of rays from the back left side lidar that detects objects on the back left side, specifically
  // the middle rays and far right and left rays
  int numBackLeftRays = sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK);
  int leftSideBoundRange = numBackLeftRays / 16;
  int midBackLeftSideRay = numBackLeftRays / 2;
  if (leftBackSideLidar.size() != 0) {
    for (int i = 0; i < leftSideBoundRange; i++) {
      backLeftSideBound.push_back(leftBackSideLidar[i]);
    }
    for (int j = midBackLeftSideRay - leftSideBoundRange / 2; j < midBackLeftSideRay + leftSideBoundRange /2; j++) {
      backLeftSideBound.push_back(leftBackSideLidar[j]);
    }
    for (int k = numBackLeftRays - leftSideBoundRange; k < numBackLeftRays; k++) {
      backLeftSideBound.push_back(leftBackSideLidar[k]);
    }
  }

  // Store vector of rays from the back right side lidar that detects objects on the back right side, specifically
  // the middle rays and far right and left rays
  int numBackRays = sdcSensorData::GetLidarNumRays(BACK);
  int backBoundRange = numBackRays / 20;
  int midBackRay = numBackRays / 2;
  if (backLidar.size() != 0) {
    for (int i = 0; i < backBoundRange; i++) {
      backRightBound.push_back(backLidar[i]);
    }
    for (int j = midBackRay - backBoundRange / 2; j < midBackRay + backBoundRange / 2; j++) {
      backMidBound.push_back(backLidar[j]);
    }
    for (int k = numBackRays - backBoundRange; k < numBackRays; k++) {
      backLeftBound.push_back(backLidar[k]);
    }
  }

  // Determines what perpendicular parking state the car should be in based on lidar data
  switch(currentPerpendicularState_) {
    // Done with perpendicular parking and sets car to stop state
    case donePark:
      llc_->StopReverse();
      llc_->Stop();
      car_->SetTurningLimit(10.0);
      car_->parkingSpotSet_ = false;
      currentState_ = STOP;
      break;

    // Pulls forward if it gets too close to anything in the back
    case frontPark:
      if (backLidar.size() != 0) {
        for (int i = 0; i < backRightBound.size(); i++) {
          if (backRightBound[i] < 1.5) {
            isSafe = false;
          }
        }
        for (int j = 0; j < backMidBound.size(); j++) {
          if (backMidBound[j] < 2.0) {
            isSafe = false;
          }
        }
        for (int k = 0; k < backLeftBound.size(); k++) {
          if (backLeftBound[k] < 1.5) {
            isSafe = false;
          }
        }
      }

      if (isSafe) {
        currentPerpendicularState_ = backPark;
      } else {
        llc_->StopReverse();
        car_->SetTargetSpeed(0.5);
        sdcAngle margin = car_->GetOrientation().FindMargin(
          car_->targetParkingAngle_);
        if (rightFrontSideLidar.size() > 0
            && leftFrontSideLidar.size() > 0
            && rightBackSideLidar.size()
            && leftBackSideLidar.size()) {
          double rightSideMargins = std::abs(
            rightFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_FRONT)/2]
            - rightBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK)/2]);
          double leftSideMargins = std::abs(
            leftFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_FRONT)/2]
            - leftBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK)/2]);
          // Begin backing up into the parking space if the car is aligned with the space
          if (margin < 0.05
              && rightSideMargins < 0.05
              && leftSideMargins < 0.05) {
            car_->parkingAngleSet_ = false;
            currentPerpendicularState_ = straightPark;
          }
        }
        car_->SetTargetDirection(car_->targetParkingAngle_);
      }
      break;

    // Backs up into parking spot until the space behind the car is good; only runs if the car
    // is aligned with the parking spot
    case straightPark:
      llc_->Reverse();
      car_->SetTargetDirection(car_->targetParkingAngle_);
      car_->SetTargetSpeed(0.5);
      if (backLidar[numBackRays / 2] < 0.5) {
        currentPerpendicularState_ = donePark;
      }
      break;

    // Temporary stop state for the car while parking to help avoid hitting anything
    case stopPark:
      llc_->Stop();
      currentPerpendicularState_ = frontPark;
      break;

    // Backs into the parking spot
    case backPark:
      // If the car is too close to anything on the back or sides, stop and fix it
      if (backLidar.size() != 0) {
        for (int i = 0; i < backRightBound.size(); i++) {
          if (backRightBound[i] < 0.7) {
            currentPerpendicularState_ = stopPark;
          }
        }
        for (int j = 0; j < backMidBound.size(); j++) {
          if (backMidBound[j] < 0.5) {
            currentPerpendicularState_ = stopPark;
          }
        }
        for (int k = 0; k < backLeftBound.size(); k++) {
          if (backLeftBound[k] < 0.7) {
            currentPerpendicularState_ = stopPark;
          }
        }
      }
      if (leftBackSideLidar.size() != 0) {
        for (int l = 0; l < backLeftSideBound.size(); l++) {
          if (leftBackSideLidar[l] < 0.25) {
            currentPerpendicularState_ = stopPark;
          }
        }
      }
      if (rightBackSideLidar.size() != 0) {
        for (int m = 0; m < backRightSideBound.size(); m++) {
          if (rightBackSideLidar[m] < 0.25) {
            currentPerpendicularState_ = stopPark;
          }
        }
      }

      // Sets a target angle for the car for when it's done parking
      if (!car_->parkingAngleSet_) {
        car_->targetParkingAngle_ = (car_->GetOrientation() - (3*PI)/2);
        car_->SetTargetDirection(2*PI - car_->targetParkingAngle_);
        car_->parkingAngleSet_ = true;
        break;
      } else {
        car_->SetTargetDirection(2*PI - car_->targetParkingAngle_);
        llc_->Reverse();
        car_->SetTargetSpeed(0.5);
      }

      // Check to see if current direction is the same as targetParkingAngle
      sdcAngle margin = car_->GetOrientation().FindMargin(
        car_->targetParkingAngle_);

      if (rightFrontSideLidar.size() > 0
          && leftFrontSideLidar.size() > 0
          && rightBackSideLidar.size()
          && leftBackSideLidar.size()) {

        double rightSideMargins = std::abs(
          rightFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_FRONT)/2]
          - rightBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK)/2]);
        double leftSideMargins = std::abs(
          leftFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_FRONT)/2]
          - leftBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK)/2]);
        if (margin < 0.05
            && rightSideMargins < 0.05
            && leftSideMargins < 0.05) {
          car_->parkingAngleSet_ = false;
          currentPerpendicularState_ = straightPark;
        }
      }
      break;
  }
}

/*
 * Parallel park algorithm
 */
void sdcHLC::ParallelPark() {
  // Get back and front lidar rays for object detection in front and in back while parking
  std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
  std::vector<double> frontLidar = sdcSensorData::GetLidarRays(FRONT);
  std::vector<double> backRightBound;
  std::vector<double> backMidBound;
  std::vector<double> backLeftBound;
  std::vector<double> frontRightBound;
  std::vector<double> frontMidBound;
  std::vector<double> frontLeftBound;
  car_->SetTurningLimit(30.0);

  // Store vector of rays from the back lidar that detect objects behind the car, specifically the middle
  // and far right and left rays
  int numBackRays = sdcSensorData::GetLidarNumRays(BACK);
  int backBoundRange = numBackRays / 20;
  int midBackRay = numBackRays / 2;
  if (backLidar.size() != 0) {
    for (int i = 0; i < backBoundRange; i++) {
      backRightBound.push_back(backLidar[i]);
    }
    for (int j = midBackRay / 2 - backBoundRange; j < midBackRay / 2 + backBoundRange; j++) {
      backMidBound.push_back(backLidar[j]);
    }
    for (int k = numBackRays - backBoundRange; k < numBackRays; k++) {
      backLeftBound.push_back(backLidar[k]);
    }
  }

  // Store vector of rays from the front lidar that detect objects in front of the car, specifically the middle
  // and far right and left rays
  int numFrontRays = sdcSensorData::GetLidarNumRays(FRONT);
  int frontBoundRange = numFrontRays / 64;
  int midFrontRay = numFrontRays / 2;
  if (frontLidar.size() != 0) {
    for (int i = 0; i < frontBoundRange; i++) {
      frontRightBound.push_back(frontLidar[i]);
    }
    for (int j = midFrontRay - frontBoundRange / 2; j < midFrontRay + frontBoundRange / 2; j++) {
      frontMidBound.push_back(frontLidar[j]);
    }
    for (int k = numFrontRays - frontBoundRange; k < numFrontRays; k++) {
      frontLeftBound.push_back(frontLidar[k]);
    }
  }

  // Determines what parallel parking state the car should be in while performing a parallel park
  switch(currentParallelState_) {
    // Drive back while turning right into parking spot
    case rightBack:
      if (!car_->parkingAngleSet_) {
        car_->targetParkingAngle_ = car_->GetOrientation();
        car_->parkingAngleSet_ = true;
      } else {
        // Turn wheels left after turning 45 degrees
        if (car_->GetOrientation() > car_->targetParkingAngle_ + PI/4) {
          currentParallelState_ = leftBack;
          break;
        }
        llc_->Reverse();
        car_->SetTargetDirection(car_->targetParkingAngle_ - PI/2);
        car_->SetTargetSpeed(0.35);
        break;
      }

    // Drive back while turning left
    case leftBack:
      if (backLidar.size() != 0 && frontLidar.size() != 0) {
        sdcAngle margin = car_->GetOrientation().FindMargin(
          car_->targetParkingAngle_);
        double spaceMargin = std::abs(
          backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
        // If the car is aligned in the parking spot, begin driving forward
        if (margin < 0.01 &&  spaceMargin < 0.05) {
          currentParallelState_ = straightForward;
          break;
        }
      }

      // If the car gets too close to anything behind it, pull forward while turning right
      if (backLidar.size() != 0) {
        for (int i = 0; i < backRightBound.size(); i++) {
          if (backRightBound[i] < 0.5) {
            currentParallelState_ = rightForward;
          }
        }
        for (int j = 0; j < backMidBound.size(); j++) {
          if (backMidBound[j] < 0.3) {
            currentParallelState_ = rightForward;
          }
        }
        for (int k = 0; k < backLeftBound.size(); k++) {
          if (backLeftBound[k] < 0.5) {
            currentParallelState_ = rightForward;
          }
        }
      }
      car_->SetTargetDirection(car_->targetParkingAngle_ + PI/2);
      llc_->Reverse();
      car_->SetTargetSpeed(0.35);
      break;

    // Drive forward while turning right
    case rightForward:
      if (backLidar.size() != 0 && frontLidar.size() != 0) {
        sdcAngle margin = car_->GetOrientation().FindMargin(
          car_->targetParkingAngle_);
        double spaceMargin = std::abs(
          backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
        if (margin < 0.01 &&  spaceMargin < 0.05) {
          currentParallelState_ = straightForward;
          break;
        }
      }

      // Back up if the car gets too close to anything in front
      if (frontLidar.size() != 0) {
        for (int i = 0; i < frontRightBound.size(); i++) {
          if (frontRightBound[i] < 0.9) {
            currentParallelState_ = leftBack;
          }
        }
        for (int j = 0; j < frontMidBound.size(); j++) {
          if (frontMidBound[j] < 0.5) {
            currentParallelState_ = leftBack;
          }
        }
        for (int k = 0; k < frontLeftBound.size(); k++) {
          if (frontLeftBound[k] < 0.9) {
            currentParallelState_ = leftBack;
          }
        }
      }
      llc_->StopReverse();
      car_->SetTargetDirection(car_->targetParkingAngle_ - PI/2);
      car_->SetTargetSpeed(0.35);
      break;

    // Pull forward while in parking spot, finishes parallel parking if the car is aligned in the spot
    // and the space in front and back are roughly the same
    case straightForward:
    {
      double frontSpace = frontLidar[numFrontRays/2];
      double backSpace = backLidar[numBackRays/2];
      if (frontSpace == backSpace) {
        currentParallelState_ = doneParallel;
        break;
      } else if (frontSpace > backSpace) {
        double spaceMargin = std::abs(frontSpace - backSpace);
        if (spaceMargin < 0.01) {
          currentParallelState_ = doneParallel;
          break;
        } else {
          car_->SetTargetDirection(car_->targetParkingAngle_);
          llc_->StopReverse();
          car_->SetTargetSpeed(0.35);
        }
      } else {
        double spaceMargin = std::abs(frontSpace - backSpace);
        if (spaceMargin < 0.01) {
          currentParallelState_ = doneParallel;
          break;
        } else {
          car_->SetTargetDirection(car_->targetParkingAngle_);
          llc_->Reverse();
          car_->SetTargetSpeed(0.35);
        }
      }
      break;
    }

    // Finished parallel parking and sets current state to stop state
    case doneParallel:
      llc_->Stop();
      llc_->StopReverse();
      car_->SetTurningLimit(10.0);
      currentState_ = STOP;
      break;
  }
}

//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcHLC::GenerateWaypoints() {
  car_->GetNSEW();
  initializeGraph();
  const int start = getFirstIntersection();
  int dest;
  for (int i = 0; i < intersections_.size(); ++i) {
    if (intersections_[i].waypoint.pos.first == destination_.first
        && intersections_[i].waypoint.pos.second == destination_.second) {
      dest = i;
    }
  }
  std::vector<int> path;
  removeStartingEdge(start);
  path = dijkstras(start, dest);
  insertWaypointTypes(path, car_->currentDir_);
  for (int i = path.size()-1; i >=0; --i) {
    WAYPOINT_VEC.push_back(intersections_[path[i]].waypoint);
  }
}

std::vector<int> sdcHLC::dijkstras(int start, int dest) {
  std::vector<int> path;
  int current;
  intersections_[start].dist = 0;
  intersections_[start].previous = -1;
  double distance;

  // initializes the unvisited_ list by placing all of start's neighbors in it
  for (int n = 0; n < intersections_[start].neighbors_pairs.size(); ++n) {
    // push back each neighbor of the start into unvisited_
    unvisited_.push_back(intersections_[start].neighbors_pairs[n].first);
    // set the distance of each neighbor to the distance of the edge
    // from start to neighbor and make neighbor previous = start
    intersections_[intersections_[start].neighbors_pairs[n].first].dist =
      intersections_[start].neighbors_pairs[n].second;
    intersections_[intersections_[start].neighbors_pairs[n].first].previous =
      intersections_[start].place;
  }

  // BFS using the unvisted FI FO vector, if unvisited_ is 0 then we have
  // visited all intersections_
  while (unvisited_.size() != 0) {
    current = unvisited_[0];
    for (int n = 0; n < intersections_[current].neighbors_pairs.size(); ++n) {
      // distance to the neighbor from current intersection
      distance = intersections_[current].neighbors_pairs[n].second;
      // if the distance of the current intersection + the distance from
      // the current intersection to neighbor is smaller than the distance
      // to neighbor, update distance and previous
      if (intersections_[intersections_[current].neighbors_pairs[n].first].dist >
          intersections_[current].dist + distance) {
        // update distance
        intersections_[intersections_[current].neighbors_pairs[n].first].dist =
          intersections_[current].dist + distance;
        // update previous
        intersections_[intersections_[current].neighbors_pairs[n].first]
          .previous = intersections_[current].place;
      }
      // if the neighbor has not been visited then push back into unvisited_
      if (intersections_[intersections_[current].neighbors_pairs[n].first].visited == 0) {
        // push back neighbor into unvisited_
        unvisited_.push_back(intersections_[current].neighbors_pairs[n].first);
      }
      // mark the current intersection as visited
      intersections_[current].visited = 1;
    }
    //pop front
    unvisited_.erase(unvisited_.begin());
  }

  //crawl backwards from dest to start to get the path
  for (int i = intersections_[dest].place; i != -1;) {
    path.push_back(i);
    i = intersections_[i].previous;
  }

  return path;
}

void sdcHLC::initializeGraph() {
  //make the sdcIntersections
  sdcIntersection aa;
  aa.place = 0;
  sdcIntersection ab;
  ab.place = 1;
  sdcIntersection ac;
  ac.place = 2;
  sdcIntersection ad;
  ad.place = 3;
  sdcIntersection ae;
  ae.place = 4;
  sdcIntersection ba;
  ba.place = 5;
  sdcIntersection bb;
  bb.place = 6;
  sdcIntersection bc;
  bc.place = 7;
  sdcIntersection bd;
  bd.place = 8;
  sdcIntersection be;
  be.place = 9;
  sdcIntersection ca;
  ca.place = 10;
  sdcIntersection cb;
  cb.place = 11;
  sdcIntersection cc;
  cc.place = 12;
  sdcIntersection cd;
  cd.place = 13;
  sdcIntersection ce;
  cd.place = 14;
  sdcIntersection da;
  cd.place = 15;
  sdcIntersection db;
  db.place = 16;
  sdcIntersection dc;
  dc.place = 17;
  sdcIntersection dd;
  dd.place = 18;
  sdcIntersection de;
  de.place = 19;
  sdcIntersection ea;
  ea.place = 20;
  sdcIntersection eb;
  eb.place = 21;
  sdcIntersection ec;
  ec.place = 22;
  sdcIntersection ed;
  ed.place = 23;
  sdcIntersection ee;
  ee.place = 24;

  //make the edges
  aa.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
  aa.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
  aa.waypoint = sdcWaypoint(0,std::pair<double,double>(0,0));

  ab.neighbors_pairs.push_back(std::pair<int, double>(0, 1));
  ab.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
  ab.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
  ab.waypoint = sdcWaypoint(0,std::pair<double,double>(0,50));


  ac.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
  ac.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
  ac.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
  ac.waypoint = sdcWaypoint(0,std::pair<double,double>(0,100));

  ad.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
  ad.neighbors_pairs.push_back(std::pair<int, double>(4, 1));
  ad.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
  ad.waypoint = sdcWaypoint(0,std::pair<double,double>(0,150));

  ae.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
  ae.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
  ae.waypoint = sdcWaypoint(0,std::pair<double,double>(0,200));

  ba.neighbors_pairs.push_back(std::pair<int, double>(0, 1));
  ba.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
  ba.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
  ba.waypoint = sdcWaypoint(0,std::pair<double,double>(50,0));

  bb.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
  bb.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
  bb.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
  bb.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
  bb.waypoint = sdcWaypoint(0,std::pair<double,double>(50,50));

  bc.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
  bc.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
  bc.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
  bc.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
  bc.waypoint = sdcWaypoint(0,std::pair<double,double>(50,100));

  bd.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
  bd.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
  bd.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
  bd.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
  bd.waypoint = sdcWaypoint(0,std::pair<double,double>(50,150));

  be.neighbors_pairs.push_back(std::pair<int, double>(4, 1));
  be.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
  be.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
  be.waypoint = sdcWaypoint(0,std::pair<double,double>(50,200));

  ca.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
  ca.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
  ca.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
  ca.waypoint = sdcWaypoint(0,std::pair<double,double>(100,0));

  cb.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
  cb.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
  cb.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
  cb.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
  cb.waypoint = sdcWaypoint(0,std::pair<double,double>(100,50));

  cc.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
  cc.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
  cc.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
  cc.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
  cc.waypoint = sdcWaypoint(0,std::pair<double,double>(100,100));

  cd.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
  cd.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
  cd.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
  cd.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
  cd.waypoint = sdcWaypoint(0,std::pair<double,double>(100,150));


  ce.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
  ce.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
  ce.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
  ce.waypoint = sdcWaypoint(0,std::pair<double,double>(100,200));


  da.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
  da.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
  da.neighbors_pairs.push_back(std::pair<int, double>(20, 1));
  da.waypoint = sdcWaypoint(0,std::pair<double,double>(150,0));


  db.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
  db.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
  db.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
  db.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
  db.waypoint = sdcWaypoint(0,std::pair<double,double>(150,50));

  dc.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
  dc.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
  dc.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
  dc.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
  dc.waypoint = sdcWaypoint(0,std::pair<double,double>(150,100));

  dd.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
  dd.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
  dd.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
  dd.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
  dd.waypoint = sdcWaypoint(0,std::pair<double,double>(150,150));

  de.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
  de.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
  de.neighbors_pairs.push_back(std::pair<int, double>(24, 1));
  de.waypoint = sdcWaypoint(0,std::pair<double,double>(150,200));

  ea.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
  ea.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
  ea.waypoint = sdcWaypoint(0,std::pair<double,double>(200,0));

  eb.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
  eb.neighbors_pairs.push_back(std::pair<int, double>(20, 1));
  eb.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
  eb.waypoint = sdcWaypoint(0,std::pair<double,double>(200,50));

  ec.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
  ec.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
  ec.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
  ec.waypoint = sdcWaypoint(0,std::pair<double,double>(200,100));

  ed.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
  ed.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
  ed.neighbors_pairs.push_back(std::pair<int, double>(24, 1));
  ed.waypoint = sdcWaypoint(0,std::pair<double,double>(200,150));

  ee.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
  ee.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
  ee.waypoint = sdcWaypoint(0,std::pair<double,double>(200,200));

  //place the intersections_ into intersections_
  intersections_ = { aa, ab, ac, ad, ae,
            ba, bb, bc, bd, be,
            ca, cb, cc, cd, ce,
            da, db, dc, dd, de,
            ea, eb, ec, ed, ee };
  //make the distance to all intersections_ infinity
  for (int i = 0; i < intersections_.size(); ++i) {
    intersections_[i].dist = std::numeric_limits<double>::infinity();
    intersections_[i].place = i;
  }
}

int sdcHLC::getFirstIntersection() {
  std::pair<double,double> firstIntr;
  int firstIntersection;

  switch(car_->currentDir_) {

    case west:
      firstIntr = {-1000,0};
      for (int i = 0; i < intersections_.size();++i) {
        if (car_->y_ < intersections_[i].waypoint.pos.second+5
            && car_->y_ > intersections_[i].waypoint.pos.second-5
            && intersections_[i].waypoint.pos.first < car_->x_ - 10
            && intersections_[i].waypoint.pos.first > firstIntr.first)
          firstIntr = intersections_[i].waypoint.pos;
      }
      break;

    case east:
      firstIntr = {1000,0};
      for (int i = 0; i < intersections_.size();++i) {
        if (car_->y_ < intersections_[i].waypoint.pos.second+5
            && car_->y_ > intersections_[i].waypoint.pos.second-5
            && intersections_[i].waypoint.pos.first > car_->x_ + 10
            && intersections_[i].waypoint.pos.first < firstIntr.first) {
          firstIntr = intersections_[i].waypoint.pos;
        }
      }
      break;

    case north:
      firstIntr = {0,1000};
      for (int i = 0; i < intersections_.size();++i) {
        if (car_->x_ < intersections_[i].waypoint.pos.first+5
            && car_->x_ > intersections_[i].waypoint.pos.first-5
            && intersections_[i].waypoint.pos.second > car_->y_ + 10
            && intersections_[i].waypoint.pos.second < firstIntr.second)
          firstIntr = intersections_[i].waypoint.pos;
      }
      break;

    case south:
      firstIntr = {0,-1000};
      for (int i = 0; i < intersections_.size();++i) {
        if (car_->x_ < intersections_[i].waypoint.pos.first+5
            && car_->x_ > intersections_[i].waypoint.pos.first-5
            && intersections_[i].waypoint.pos.second < car_->y_ - 10
            && intersections_[i].waypoint.pos.second > firstIntr.second)
          firstIntr = intersections_[i].waypoint.pos;
      }
      break;
  }

  for (int i = 0; i < intersections_.size();i++) {
    if (firstIntr.first == intersections_[i].waypoint.pos.first
        && firstIntr.second == intersections_[i].waypoint.pos.second) {
      firstIntersection = i;
      break;
    }
  }
  return firstIntersection;
}

void sdcHLC::insertWaypointTypes(std::vector<int> path, Direction startDir) {
  Direction curDir = startDir;
  Direction nextDir;
  int current;
  int next;
  // get the direction the car heads in from the current intersection to
  // the next one
  for (int i = path.size() - 1; i > 0; i--) {
  current = path[i];
  next = path[i - 1];
  if (next - current == size) {
    nextDir = east;
  } else if (current - next == size) {
    nextDir = west;
  } else if (next - current == 1) {
    nextDir = north;
  } else if (current - next == 1) {
    nextDir = south;
  }

  switch (curDir) {
    case north:
      switch (nextDir) {
        case north:
          intersections_[current].waypoint.waypointType = WaypointType_DriveStraight;
          break;
        case east:
          intersections_[current].waypoint.waypointType = WaypointType_TurnRight;
          break;
        case west:
          intersections_[current].waypoint.waypointType = WaypointType_TurnLeft;
        case south:
          break;
      }
      break;
    case south:
      switch (nextDir) {
        case south:
          intersections_[current].waypoint.waypointType = WaypointType_DriveStraight;
          break;
        case east:
          intersections_[current].waypoint.waypointType = WaypointType_TurnLeft;
          break;
        case west:
          intersections_[current].waypoint.waypointType = WaypointType_TurnRight;
        case north:
          break;
      }
      break;
    case east:
      switch (nextDir) {
        case north:
          intersections_[current].waypoint.waypointType = WaypointType_TurnLeft;
          break;
        case south:
          intersections_[current].waypoint.waypointType = WaypointType_TurnRight;
          break;
        case east:
          intersections_[current].waypoint.waypointType = WaypointType_DriveStraight;
        case west:
          break;
      }
      break;
    case west:
      switch (nextDir) {
        case north:
          intersections_[current].waypoint.waypointType = WaypointType_TurnRight;
          break;
        case south:
          intersections_[current].waypoint.waypointType = WaypointType_TurnLeft;
          break;
        case west:
          intersections_[current].waypoint.waypointType = WaypointType_DriveStraight;
        case east:
          break;
      }
      break;
  }
  curDir = nextDir;
  }
  intersections_[path[0]].waypoint.waypointType = WaypointType_Stop;
}

void sdcHLC::removeStartingEdge(int start) {
  Direction dir = east;
  switch (dir) {
    case north:
      for (int n = 0; n < intersections_[start].neighbors_pairs.size(); ++n) {
        if (intersections_[start].neighbors_pairs[n].first == start - 1) {
          intersections_[start].neighbors_pairs[n].second =
            std::numeric_limits<double>::infinity();
        }
      }
      break;
    case south:
      for (int n = 0; n < intersections_[start].neighbors_pairs.size(); ++n) {
        if (intersections_[start].neighbors_pairs[n].first == start + 1) {
          intersections_[start].neighbors_pairs[n].second =
            std::numeric_limits<double>::infinity();
        }
      }
      break;
    case east:
      for (int n = 0; n < intersections_[start].neighbors_pairs.size(); ++n) {
        if (intersections_[start].neighbors_pairs[n].first == start - size) {
          intersections_[start].neighbors_pairs[n].second =
            std::numeric_limits<double>::infinity();
        }
      }
      break;
    case west:
      for (int n = 0; n < intersections_[start].neighbors_pairs.size(); ++n) {
        if (intersections_[start].neighbors_pairs[n].first == start + size) {
          intersections_[start].neighbors_pairs[n].second =
            std::numeric_limits<double>::infinity();
        }
      }
      break;
  }
}


void sdcHLC::update() {
  llc_->update();
}
