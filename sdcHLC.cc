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
#include "CameraPlugin.hh"

using namespace gazebo;

std::vector<sdcWaypoint> WAYPOINT_VEC;

sdcHLC::sdcHLC(sdcCar* car): car_(car) {
  llc_ = new sdcLLC(car_);

  // Initialize state enums
  DEFAULT_STATE = WAYPOINT;
  currentState_ = DEFAULT_STATE;

  roadState_ = FOLLOW_16;

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

bool sdcHLC::IsBackToLane() {
  double angle = dataProcessing::GetPassPointAngle();
  // printf("Angle: %f\n", angle);
  return (angle >= 2.46) ? true : false;
}

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

  bool isDangerousObj =
    dataProcessing::IsNearbyObject()
    && IsObjectOnCollisionCourse(dataProcessing::GetNearbyObject());

  if (isDangerousObj) {
    roadState_ = AVOID_16;
  } else if ((!dataProcessing::IsNearbyObject() && roadState_ == AVOID_16) || (roadState_ == RETURN_16)) {
    roadState_ = RETURN_16;
  } else if (!dataProcessing::IsNearbyObject()) {
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
      // printf("PASS POINT! Go to (%f,%f)\n", targetPoint.x, targetPoint.y);
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
  // printf("pathdist_: %f\n", pathDist_);
  // double lookaheadDistance = ScaledLookaheadDistance();
  double lookaheadDistance = 20;

  if (llc_->BeyondPath(pathDist_ + lookaheadDistance)) {
    llc_->GenerateNewDubins();
    pathDist_ = 0;
  }
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
  double possibleCollisionTime = DoMaximumRadiiCollide(obj);
  // return possibleCollisionTime != -1;
  if (possibleCollisionTime != -1) {
    // printf("radii collide!\n");
    return true;
  }
  return false;
  // return possibleCollisionTime != -1
  //   && DoAccurateVehicleShapesCollide(obj, possibleCollisionTime);
}

// /*
//  * Checks if the most pessimistic bounding boxes collide; that
//  */
// bool sdcHLC::DoMaximumBoundingBoxesCollide(const sdcVisibleObject* obj) const {
//   double maxTime = car_->GetMaxSafeTime();
//   math::Vector2d futureCarPos = GetPositionAtTime(maxTime);
//   math::Vector2d futureObjPos = obj->GetProjectedPositionAtTime(maxTime);
//
//   sdcBoundingBox carRect = sdcBoundingBox(
//     fmin(car_->GetBackLeft(), GetBackLeft(futureCarPos, GetAngleAtTime(maxTime).angle),
//     fmin(car_->GetBackLeft(), GetBackLeft(futureCarPos, GetAngleAtTime(maxTime).angle),
//     fmax(car_->GetBackLeft(), GetBackLeft(futureCarPos, GetAngleAtTime(maxTime).angle),
//     fmax(car_->GetBackLeft(), GetBackLeft(futureCarPos, GetAngleAtTime(maxTime).angle));
//
//   // TODO: determine the
//   sdcBoundingBox objRect = sdcBoundingBox(
//     fmin(obj->GetCenterPoint().x, futureObjPos.x),
//     fmin(obj->GetCenterPoint().y, futureObjPos.y),
//     fmax(obj->GetCenterPoint().x, futureObjPos.x),
//     fmax(obj->GetCenterPoint().y, futureObjPos.y));
//
//   return carRect.DoesIntersect(objRect);
// }

/*
 * Returns true if the distance between the car and the dangerous object is
 * ever within (max_radius_car + max_radius_obj) along their projected paths.
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
 * Returns true if the distance between the car and obj at is within
 * (max_radius_car + max_radius_obj) at the given time
 */
bool sdcHLC::DoMaximumRadiiCollideAtTime(const sdcVisibleObject* obj,
                                         double time) const {
  sdcBoundingCircle selfCircle = sdcBoundingCircle(
    GetPositionAtTime(time),
    pythag_thm(car_->width_, car_->length_)
  );

  cv::Point2d carPos = dataProcessing::GetCarLocation();
  cv::Point2d objLeft = obj->GetLeftPos(carPos);
  cv::Point2d objRight = obj->GetRightPos(carPos);
  cv::Point2d objCenter = cv::Point2d((objLeft.x + objRight.x) / 2,
                                      (objLeft.y + objRight.y) / 2);

  sdcBoundingCircle objCircle = sdcBoundingCircle(
    objCenter,
    coord_distance(objCenter, objRight)
  );

  // if (selfCircle.DoesIntersect(objCircle)) {
  //   printf ("collision!\n");
  // } else {
  //   printf("no collision\n");
  // }
  // printf("  objLeft: (%f, %f)\n", objLeft.x, objLeft.y);
  // printf("  carPos:  (%f, %f)\n\n", carPos.x, carPos.y);

  return selfCircle.DoesIntersect(objCircle);
}

/*
 * Returns true if accurate shape depictions of the car and the object
 * ever intersect along their projected paths.
 * TODO: test the numbers (10, 100) to see if they are reasonable at all
 */
bool sdcHLC::DoAccurateVehicleShapesCollide(const sdcVisibleObject* obj,
                                            double possibleCollisionTime) const {
  int numTests = 20;
  for (int i = (possibleCollisionTime - 1) * numTests; i < (possibleCollisionTime + 4) * numTests; i++) {
    if (DoAccurateVehicleShapesCollideAtTime(obj, ((double)i) / numTests)) {
      // printf("  accurate shapes collide\n");
      return true;
    }
  }
  // printf("  shapes do not collide\n");
  return false;
}

/*
 * Returns true if accurate shape depictions of the car and the object
 * intersect at the given time
 */
bool sdcHLC::DoAccurateVehicleShapesCollideAtTime(const sdcVisibleObject* obj,
                                                  double time) const {
  cv::Point2d selfPos = GetPositionAtTime(time);
  sdcRotatedBoundingBox selfBox = sdcRotatedBoundingBox(
    cv::Point2d(car_->x_, car_->y_),
    car_->width_ * 1.2, car_->length_ * 1.2,
    GetAngleAtTime(time).angle
  );

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
 * Returns the projected position of the car at the given time.
 */
cv::Point2d sdcHLC::GetPositionAtTime(double time) const {
  double distance = car_->GetSpeed() * time;
  cv::Point2d carPos = dataProcessing::GetCarLocation();
  cv::Point2d newPos = carPos;
  newPos.x += distance;
  return rotate_generic(newPos, carPos, car_->GetOrientation().angle);
  // return llc_->GetDubinsPoint(distance, false);
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
