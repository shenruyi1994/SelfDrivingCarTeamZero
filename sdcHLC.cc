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
#include "sdcUtils.hh"
#include "sdcLLC.hh"
#include "sdcRotatedBoundingBox.hh"
#include "Waypoints.hh"
#include "CameraPlugin.hh"

using namespace gazebo;

sdcHLC::sdcHLC(sdcCar* car): car_(car) {
  llc_ = new sdcLLC(car_);

  roadState_ = FOLLOW_16;
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

  // only run this code every 10 milliseconds, rather than every Gazebo update
  common::Time curTime = car_->model_->GetWorld()->GetSimTime();
  if (curTime.Double() - lastUpdateTime_.Double() < .01) {
    return;
  } else {
    lastUpdateTime_ = common::Time(curTime);
  }

  bool isObj = dataProcessing::IsNearbyObject();
  bool isDangerousObj = isObj && IsObjectOnCollisionCourse(dataProcessing::GetNearbyObject());

  if (isDangerousObj) {
    roadState_ = AVOID_16;
  } else if ((!isObj && roadState_ == AVOID_16) || (roadState_ == RETURN_16)) {
    roadState_ = RETURN_16;
  } else if (!isObj) {
    roadState_ = FOLLOW_16;
  }

  switch (roadState_) {
    case WAIT_16:
      // increment timer, pass eventually
      car_->SetTargetSpeed(0);
      break;

    case PASS_16:
      // pass a car that we've been waiting behind, similar to AVOID_16 in that
      // we need to find a point to the side ang get around it
      break;

    case STOP_16: // break hard
      car_->SetTargetSpeed(0);
      car_->SetBrakeRate(10);
      break;

    case AVOID_16:
      AvoidObstacle();
      break;

    case RETURN_16:
      BackToLane();
      break;

    case FOLLOW_16: // fall through, follow waypoints with no obstacle
    default:
      FollowWaypoints();
  }

  // Attempt to turn towards the target direction and match target speed
  MatchTargetDirection();
  MatchTargetSpeed();
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
 * Drive along a road or other surface, following waypoints. Rather than
 * directly following a path, aim for a point in front of the vehicle. This
 * less accurately follows the waypoints but provides a smoother transition
 * along sharp turns in the curve.
 */
void sdcHLC::FollowWaypoints() {
  car_->SetTargetSpeed(3);
  cv::Point2d targetPoint;

  if(roadState_ == FOLLOW_16){
    targetPoint = FindDubinsTargetPoint();
  } else {

    // If in RETURN_16 state, but CameraPlugin hasn't finished computing targetPoint
    if (targetPoint.x == 0 && targetPoint.y == 0 && dataProcessing::GetPassPointAngle() == 0){
      // Do as it does in FOLLOW_16 state
      targetPoint = FindDubinsTargetPoint();
    } else {
      targetPoint = dataProcessing::GetPassPoint();
      car_->SetTargetPoint(targetPoint);
      if (IsBackToLane()) {
        roadState_ = FOLLOW_16;
        CameraPlugin::SetReturnMode(false);
        dataProcessing::UpdatePassPoint(cv::Point(0.0));
        dataProcessing::UpdatePassPointAngle(0);
      }
    }
  }
  car_->SetTargetPoint(targetPoint);
}

/*
 * ===== BREAKS THE CAR =====
 * Calculate the turning angle necessary to reach the provided point, and then
 * set the wheel angle correctly. Based on the tuned pure pursuit algorithm
 * found in this paper:
 * http://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
 *
 * This is a better way to follow a more complicated path (like a spline), but
 * as of right now it makes the car model explode sometimes.
 */
void sdcHLC::AngleWheelsTowardsTarget(const math::Vector2d& target) {
  sdcAngle alpha = car_->AngleToTarget(target);
  double numerator = 2 * WHEEL_BASE * sin(alpha.angle);
  double denominator = lookaheadScalor_ * car_->GetSpeed();

  sdcAngle directionAngle = atan2(numerator, denominator);
}

/*
 * Returns the point along the dubins path that the car should be following.
 *
 * TODO: fix the GetDubinsPoint() bug where it appears to return points in
 *       reverse order or something
 */
cv::Point2d sdcHLC::FindDubinsTargetPoint() {
  cv::Point2d location = cv::Point2d(car_->x_, car_->y_);
  // double lookaheadDistance = ScaledLookaheadDistance();
  double lookaheadDistance = 20;

  cv::Point2d tempTarget = llc_->GetDubinsPoint(lookaheadDistance);
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
 * In avoidance, the car's only concern is not hitting objects. Provides a couple emergency methods, one
 * for stopping and one for swerving. Also provides a navigation case for maneuvering around objects in front
 * of the car
 */
void sdcHLC::AvoidObstacle() {
  cv::Point2d targetPoint = dataProcessing::getObstacleCoords();
  // printf("AVOIDANCE STATE! Go to (%f,%f)\n", targetPoint.x, targetPoint.y);
  car_->SetTargetPoint(targetPoint);
}

void sdcHLC::BackToLane(){
  CameraPlugin::SetReturnMode(true);
  FollowWaypoints();
}

bool sdcHLC::IsBackToLane() {
  return dataProcessing::GetPassPointAngle() >= 2.46;
}

//////////////////////////////////////////
//////////////////////////////////////////
//  BEGIN COLLISION DETECTION FUNCTIONS //
//////////////////////////////////////////
//////////////////////////////////////////

/*
 * ===== NOT CURRENTLY IN USE =====
 * This method aims to decide how to avoid; i.e. to stop or slow down instead
 * going around. Some example scenarios include a large object across the
 * entire road, or if the object in front is a car going slightly slower than
 * our vehicle.
 */
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
 * ===== NEEDS TESTING =====
 * Returns true if the car is able to stop before hitting the object.
 * Uses the equation d = v^2 / 20
 * Should be tested more to determine the best constant for the denominator.
 */
bool sdcHLC::CanStopBeforeObject(const sdcVisibleObject* obj) const {
  return obj->Dist() > sqrt(car_->GetSpeed()) / 20;
}

/*
 * Checks if the object is on a collision course with the car.
 */
bool sdcHLC::IsObjectOnCollisionCourse(const sdcVisibleObject* obj) const {
  return DoMaximumBoundingBoxesCollide(obj) && DoMaximumRadiiCollide(obj) != -1;
}

/*
 * Checks if the most pessimistic bounding boxes collide; that is, if the
 * minimum bounding box that encompasses the car over the next MaxSafeTime
 * seconds intersects with that of the obstacle.
 *
 * =====================================
 * =========== W A R N I N G ===========
 * =====================================
 * This is weirdly non-deterministic -- likely a gazebo rendering issue, but
 * still worth being aware of.
 */
bool sdcHLC::DoMaximumBoundingBoxesCollide(const sdcVisibleObject* obj) const {
  double maxTime = car_->GetMaxSafeTime();
  cv::Point2d carPos = cv::Point2d(car_->x_, car_->y_);
  cv::Point2d futureCarPos = GetPositionAtTime(maxTime);

  sdcBoundingBox carRect = sdcBoundingBox(
    fmin(carPos.x - .5*car_->length_, futureCarPos.x - .5*car_->length_), // left
    fmin(carPos.y + .5*car_->length_, futureCarPos.y + .5*car_->length_), // top
    fabs(carPos.x - futureCarPos.x) + car_->width_, // width
    fabs(carPos.y - futureCarPos.y) + car_->width_  // height
  );

  cv::Point2d objLeft = obj->GetLeftPos(carPos);
  cv::Point2d objRight = obj->GetRightPos(carPos);
  double objWidth = coord_distance(objLeft, objRight);
  cv::Point2d objPos = .5 * (objLeft + objRight);

  // TODO: use GetProjectedPositionAtTime() when we have moving obstacles
  // cv::Point2d futureObjPos = to_point(obj->GetProjectedPositionAtTime(maxTime));
  cv::Point2d futureObjPos = objPos;

  sdcBoundingBox objRect = sdcBoundingBox(
    fmin(objPos.x - .5*objWidth, futureObjPos.x - .5*objWidth), // left
    fmin(objPos.y + .5*objWidth, futureObjPos.y + .5*objWidth), // top
    fabs(objPos.x - futureObjPos.x) + objWidth, // width
    fabs(objPos.y - futureObjPos.y) + objWidth  // height
  );

  return carRect.DoesIntersect(objRect);
}

/*
 * Returns true if the distance between the car and the dangerous object is
 * ever within (max_radius_car + max_radius_obj) along their projected paths.
 *
 * TODO: optimize the range of times we look through
 */
double sdcHLC::DoMaximumRadiiCollide(const sdcVisibleObject* obj) const {
  int numTests = 20;
  double maxSafeTime = 30 / car_->GetSpeed();
  for (int i = 0; i < maxSafeTime * numTests; i++) {
    if (DoMaximumRadiiCollideAtTime(obj, ((double)i) / numTests)) {
      return ((double)i) / numTests;
    }
  }
  return -1;
}

/*
 * Returns true if the distance between the car and obj is within
 * (max_radius_car + max_radius_obj) at the given time
 */
bool sdcHLC::DoMaximumRadiiCollideAtTime(const sdcVisibleObject* obj,
                                         double time) const {
  // Generates the minimum bounding circle around the car
  sdcBoundingCircle selfCircle = sdcBoundingCircle(
    GetPositionAtTime(time),
    pythag_thm(car_->width_, car_->length_)
  );

  // Generates the minimum bounding circle around the obstacle, determined by
  // the furthest left and right lidar rays that detect it.
  cv::Point2d carPos = dataProcessing::GetCarLocation();
  cv::Point2d objLeft = obj->GetLeftPos(carPos);
  cv::Point2d objRight = obj->GetRightPos(carPos);
  cv::Point2d objCenter = cv::Point2d((objLeft.x + objRight.x) / 2,
                                      (objLeft.y + objRight.y) / 2);

  sdcBoundingCircle objCircle = sdcBoundingCircle(
    objCenter,
    coord_distance(objCenter, objRight)
  );

  return selfCircle.DoesIntersect(objCircle);
}

/*
 * ===== NOT CURRENTLY IN USE =====
 * Returns true if accurate shape depictions of the car and the object
 * ever intersect along their projected paths.
 *
 * TODO: optimize the range of times we look through
 */
bool sdcHLC::DoAccurateVehicleShapesCollide(const sdcVisibleObject* obj,
                                            double possibleCollisionTime) const {
  int numTests = 20;
  for (int i = (possibleCollisionTime - 1) * numTests; i < (possibleCollisionTime + 4) * numTests; i++) {
    if (DoAccurateVehicleShapesCollideAtTime(obj, ((double)i) / numTests)) {
      return true;
    }
  }
  return false;
}

/*
 * ===== NOT CURRENTLY IN USE =====
 * Returns true if accurate shape depictions of the car and the object
 * intersect at the given time.
 *
 * We did not have time to test the method throughly. It appears
 */
bool sdcHLC::DoAccurateVehicleShapesCollideAtTime(const sdcVisibleObject* obj,
                                                  double time) const {
  cv::Point2d selfPos = GetPositionAtTime(time);
  sdcRotatedBoundingBox selfBox = sdcRotatedBoundingBox(
    cv::Point2d(car_->x_, car_->y_),
    car_->width_ * 1.2, car_->length_ * 1.2,
    GetAngleAtTime(time).angle
  );

  // Imagines the obstacle as a circle. With more advanced detection techniques
  // this could be improved to imagine it as a rectangle or something more
  // complex.
  cv::Point2d objLeft = obj->GetLeftPos(dataProcessing::GetCarLocation());
  cv::Point2d objRight = obj->GetRightPos(dataProcessing::GetCarLocation());
  cv::Point2d objCenter = cv::Point2d((objLeft.x + objRight.x) / 2,
                                      (objLeft.y + objRight.y) / 2);

  sdcBoundingCircle objCircle = sdcBoundingCircle(
    objCenter,
    coord_distance(objCenter, objRight)
  );
  return selfBox.DoesIntersect(objCircle);
}

/*
 * ===== NEEDS WORK =====
 * Returns the projected position of the car at the given time.
 *
 * TODO: modify to use llc_->GetDubinsPoint(). We were having a bug around the
 * time of code freeze and GetDubinsPoint() was possibly storing waypoints in
 * reverse order. Therefore this works perfectly when going straight, but will
 * be off for tight turns.
 */
cv::Point2d sdcHLC::GetPositionAtTime(double time) const {
  double distance = car_->GetSpeed() * time;
  cv::Point2d carPos = dataProcessing::GetCarLocation();
  cv::Point2d newPos = carPos;
  newPos.x += distance;

  return rotate_generic(newPos, carPos, car_->GetOrientation().angle);
}

/*
 * Returns the projected angle of the car at the given time.
 */
sdcAngle sdcHLC::GetAngleAtTime(double time) const {
  double distance = car_->GetSpeed() * time;
  return llc_->GetDubinsAngle(distance, false);
}

/*
 * Returns the angle between the car and the vehicle at the time of collision,
 * allowing for improved avoidance decisionmaking.
 */
sdcAngle sdcHLC::GetCollisionAngleAtTime(const sdcVisibleObject* obj,
                                         double time) const {
  return sdcAngle();
}

void sdcHLC::update() {
  llc_->update();
}
