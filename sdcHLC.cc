#include "sdcHLC.hh"

#include <vector>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "globals.hh"
#include "sdcCar.hh"
#include "sdcIntersection.hh"
#include "sdcLLC.hh"
#include "Waypoints.hh"

using namespace gazebo;

sdcHLC::sdcHLC(sdcCar* car): car_(car) {
    this->llc_ = new sdcLLC(car_);
    waypoints_ = new Waypoints();

    // Initialize state enums
    this->DEFAULT_STATE = WAYPOINT;
    this->currentState = DEFAULT_STATE;

    this->currentPerpendicularState_ = backPark;
    this->currentParallelState_ = rightBack;
    this->currentAvoidanceState_ = notAvoiding;
}

sdcHLC::~sdcHLC() {
    delete llc_;
    delete waypoints_;
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

    // If not in avoidance, check if we should start following the thing
    // in front of us. If following is done, kick out to default state
    if (this->currentState != INTERSECTION && this->currentState != AVOIDANCE) {
        // If there's a stop sign, assume we're at an intersection
        if (this->car_->ignoreStopSignsCounter == 0 && sdcSensorData::stopSignFrameCount > 5) {
            this->currentState = INTERSECTION;
        }

        // If something is ahead of us, default to trying to follow it
        if (this->car_->ObjectDirectlyAhead()) {
            this->currentState = FOLLOW;
        } else if (this->currentState == FOLLOW && !this->car_->isTrackingObject) {
            this->currentState = this->DEFAULT_STATE;
        }

        // Look for objects in danger of colliding with us, react appropriately
        if (this->car_->ObjectOnCollisionCourse()) {
            this->currentState = AVOIDANCE;
        }
    }

    this->car_->ignoreStopSignsCounter = fmax(this->car_->ignoreStopSignsCounter - 1, 0);


    // Possible states: stop, waypoint, intersection, follow, avoidance
    switch(this->currentState) {
        // Final state, car is finished driving
        case STOP:
            this->llc_->Stop();
            break;

        // Default state; drive straight to target location
        case  WAYPOINT:
            // Handle lane driving

            this->llc_->Accelerate();
            // this->llc_->Stop();
            //this->WaypointDriving(WAYPOINT_VEC);
            break;

        // At a stop sign, performing a turn
        case INTERSECTION:
            if (this->car_->stoppedAtSign && this->car_->stationaryCount > 2000) {
                this->currentState = this->DEFAULT_STATE;
                this->car_->ignoreStopSignsCounter = 3000;
            } else if (this->car_->stoppedAtSign && this->car_->GetSpeed() < 0.5) {
                this->car_->stationaryCount++;
            } else if (!this->car_->stoppedAtSign && sdcSensorData::sizeOfStopSign > 6000) {
                this->llc_->Stop();
                this->car_->stoppedAtSign = true;
                this->car_->stationaryCount = 0;
            }

        break;

        // Follows object that is going in same direction/towards same target
        case FOLLOW:
            this->Follow();
            // Handle lane driving
            break;

        // Smarter way to avoid objects; stopping, swerving, etc.
        case AVOIDANCE:
            // Cases: stop, swerve, go around
            this->Avoidance();
            break;

        // Parks the car
        case PARKING:
            this->PerpendicularPark();
            // this->ParallelPark();
            break;
    }

    // Attempts to turn towards the target direction
    this->MatchTargetDirection();
    // Attempts to match the target speed
    this->MatchTargetSpeed();
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcHLC::MatchTargetDirection() {
    sdcAngle directionAngleChange = this->car_->GetDirection() - this->car_->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.WithinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // The steering amount scales based on how far we have to turn, with upper and lower limits
        double proposedSteeringAmount =
            fmax(fmin(-this->car_->turningLimit * tan(directionAngleChange.angle/-2),
                      this->car_->turningLimit), -this->car_->turningLimit);

        // When reversing, steering directions are inverted
        if (!this->car_->reversing) {
            this->car_->SetTargetSteeringAmount(proposedSteeringAmount);
        } else {
            this->car_->SetTargetSteeringAmount(-proposedSteeringAmount);
        }
    }

    // Check if the car needs to steer, and apply a small turn in the corresponding direction
    if (!(std::abs(this->car_->targetSteeringAmount - this->car_->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
        if (this->car_->steeringAmount < this->car_->targetSteeringAmount) {
            this->car_->steeringAmount =
                this->car_->steeringAmount + STEERING_ADJUSTMENT_RATE;
        } else {
            this->car_->steeringAmount =
                this->car_->steeringAmount - STEERING_ADJUSTMENT_RATE;
        }
    }
}

/*
 * Attempts to match the current target speed
 */
void sdcHLC::MatchTargetSpeed() {
    // Invert all the values if the car should be moving backwards
    int dirConst = this->car_->reversing ? -1 : 1;

    // If the car is moving the wrong direction or slower than the target speed, press on the gas
    if ((this->car_->reversing && this->car_->IsMovingForwards())
            || (!this->car_->reversing && !this->car_->IsMovingForwards())
            || (this->car_->GetSpeed() < this->car_->targetSpeed)) {
        this->car_->gas = 1.0 * dirConst;
        this->car_->brake = 0.0;
    } else if (this->car_->GetSpeed() > this->car_->targetSpeed) {
        // If the car is moving faster than the target speed, brake to slow down
        this->car_->gas = 0.0;
        if (this->car_->reversing != this->car_->IsMovingForwards()) {
            this->car_->brake = -2.0 * dirConst;
        } else {
            // If the car is drifting in the opposite direction it should be, don't brake
            // as this has the side effect of accelerating the car in the opposite direction
            this->car_->brake = 0.0;
        }
    }
}

/*
 * Drive from point to point in the given list
 */
void sdcHLC::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
    int progress = this->car_->waypointProgress;
    if (progress < WAYPOINT_VEC.size()) {
        // Pull the next waypoint and set the car to drive towards it


        this->llc_->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->car_->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->car_->y,2));
        if (distance < 7) {
            this->car_->turning = true;
        }
        if (this->car_->turning == true) {
            this->car_->SetTurningLimit(20);
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
            sdcAngle targetAngle = this->car_->AngleToTarget(nextTarget);
            this->car_->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
    } else if (this->car_->isFixingParking) {
        this->car_->isFixingParking = false;
        this->currentState = PARKING;
        this->currentPerpendicularState_ = straightPark;
    } else {
        this->currentState = STOP;
    }
}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcHLC::LanedDriving() {
    int lanePos = sdcSensorData::LanePosition();
    this->car_->SetTurningLimit(sdcSensorData::GetNewSteeringMagnitude());
    if (!(lanePos > 320 || lanePos < -320)) {
        // It's beautiful don't question it
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->car_->SetTargetDirection(this->car_->GetDirection() + laneWeight);
    }
}

/*
 * Car follows an object directly in front of it and slows down to stop when it starts to get close
 */
void sdcHLC::Follow() {
    // There's nothing in front of the car, so break out of follow
    if (this->car_->frontObjects.size() == 0) {
        this->car_->isTrackingObject = false;
        this->currentState = this->DEFAULT_STATE;
        return;
    }

    // The default object to follow is directly in front of the car, the max range away
    sdcVisibleObject tracked = sdcVisibleObject(
        sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)),
        sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)),
        sdcSensorData::GetLidarMaxRange(FRONT));

    // Already tracking an object, find it again
    if (this->car_->isTrackingObject) {
        bool foundTrackedObject = false;
        for (int i = 0; i < this->car_->frontObjects.size(); i++) {
            sdcVisibleObject obj = this->car_->frontObjects[i];
            if (obj.IsTracking()) {
                tracked = obj;
                foundTrackedObject = true;
                break;
            }
        }
        if (!foundTrackedObject) {
            this->car_->isTrackingObject = false;
            return;
        }
    } else {
        // Not tracking an object, find one that's in front of the car
        // and start tracking it
        for (int i = 0; i < this->car_->frontObjects.size(); i++) {
            sdcVisibleObject obj = this->car_->frontObjects[i];
            if (this->car_->IsObjectDirectlyAhead(obj)) {
                tracked = obj;
                tracked.SetTracking(true);
                this->car_->isTrackingObject = true;
                this->car_->frontObjects[i] = tracked;
                break;
            }
        }
    }

    // After the above loops, if not following anything just return
    if (!this->car_->isTrackingObject) return;

    math::Vector2d objCenter = tracked.GetCenterPoint();
    double objSpeed = tracked.GetEstimatedYSpeed();

    // Scale our speed based on how far away the tracked object is
    // The equation is 'scaledSpeed = (objY - 10)^3 / 2000.' which
    // gives a scaled speed of 0 at y=10 and +-0.5 at y=20, y=0 respectively
    double scaledSpeed = pow(objCenter.y - 10, 3) / 2000.;

    // Adjust the target speed based on the speed of the object, our speed,
    // and the above calculated scaled speed
    double newTargetSpeed = objSpeed + this->car_->GetSpeed() + scaledSpeed;
    this->car_->SetTargetSpeed(newTargetSpeed);

    // If the new target speed is sufficiently low, count the car as stationary
    if (newTargetSpeed < 0.3) {
        this->car_->stationaryCount++;
    } else {
        this->car_->stationaryCount = 0;
    }

    // If the car has been stationary for sufficiently long, stop following and start
    // trying to navigate around the object in front of it
    if (this->car_->stationaryCount > 2000) {
        this->currentState = AVOIDANCE;
        this->currentAvoidanceState_ = navigation;
    }

    // Set the direction of the car to be angled at the tracked object
    if (objCenter.x != 0) {
        this->car_->SetTargetDirection(
            this->car_->GetOrientation()
            - sdcAngle(PI / 2.)
            + sdcAngle(atan2(objCenter.y, objCenter.x)));
    } else {
        this->car_->SetTargetDirection(this->car_->GetOrientation());
    }
}

/*
 * In avoidance, the car's only concern is not hitting objects. Provides a couple emergency methods, one
 * for stopping and one for swerving. Also provides a navigation case for maneuvering around objects in front
 * of the car
 */
void sdcHLC::Avoidance() {
    // If there's nothing in front of the car and it's not in the middle
    // of a navigation operation, exit the avoidance state
    if (this->car_->frontObjects.size() == 0 && !this->car_->trackingNavWaypoint) {
        this->currentState = this->DEFAULT_STATE;
        this->currentAvoidanceState_ = notAvoiding;
        return;
    }

    // Get lists of objects that are moving quickly towards us,
    // and objects that are close to us
    std::vector<sdcVisibleObject> fastObjects, furiousObjects;
    if (this->car_->frontObjects.size() > 0) {
        for (int i = 0; i < this->car_->frontObjects.size(); i++) {
            if (this->car_->IsObjectTooFast(this->car_->frontObjects[i])) {
                fastObjects.push_back(this->car_->frontObjects[i]);
            }
            if (this->car_->IsObjectTooFurious(this->car_->frontObjects[i])) {
                furiousObjects.push_back(this->car_->frontObjects[i]);
            }
        }
    }

    // For emergency swerve, check which side the object is coming from so
    // we can go away from it
    bool isObjectOnRight = true;

    // Objects moving relatively quickly towards the car are the highest priority. If any
    // of these exist, react accordingly
    if (fastObjects.size() > 0) {
        bool setState = false;
        for (int i = 0; i < fastObjects.size(); i++) {
            // If the object is moving faster than the car is, or the car is moving significantly faster than the object,
            // try and swerve as there isn't enough time to stop
            double objSpeed = sqrt(
                pow(fastObjects[i].GetEstimatedXSpeed(), 2)
                + pow(fastObjects[i].GetEstimatedYSpeed() - this->car_->GetSpeed(), 2));

            if (objSpeed > this->car_->GetSpeed()
                    || this->car_->GetSpeed() > objSpeed + 4) {
                this->currentAvoidanceState_ = emergencySwerve;
                if (fastObjects[i].GetCenterPoint().x < 0) {
                  isObjectOnRight = false;
                }
                setState = true;
                break;
            }
        }

        // If the state hasn't been set to swerve, the car should be able to stop and thus
        // avoid a collision
        if (!setState) {
            this->currentAvoidanceState_ = emergencyStop;
        }
    } else if (furiousObjects.size() > 0) {
        // There are objects very close to the car, but not necessarily in danger of running into
        // it. Try and navigate around them
        this->currentAvoidanceState_ = navigation;
    } else if (this->currentAvoidanceState_ != navigation
            && this->currentAvoidanceState_ != emergencyStop) {
        // No dangerous objects were found, and the car is not in the middle of navigating around
        // objects in front of it. Exit to default state
        this->currentAvoidanceState_ = notAvoiding;
        this->currentState = this->DEFAULT_STATE;
        return;
    }

    switch(this->currentAvoidanceState_) {
        // Stop, hard.
        case emergencyStop:
            this->llc_->Stop();
            this->car_->SetBrakeRate(10);
            break;

        // Make an emergency turn and attempt to accelerate past
        // the incoming danger
        case emergencySwerve:
            if (isObjectOnRight) {
              this->car_->SetTargetDirection(this->car_->GetOrientation() + PI/4);
            } else {
              this->car_->SetTargetDirection(this->car_->GetOrientation() - PI/4);
            }
            this->car_->SetTargetSpeed(10);
            this->car_->SetAccelRate(10);
            break;

        // Carefully maneuver around perceived obstacles
        case navigation:
        {
            // Set the target speed very low, and if the car is moving
            // sufficiently slowly increase the rate we can turn
            this->car_->SetTargetSpeed(1);
            if (this->car_->GetSpeed() < 2) {
                this->car_->SetTurningLimit(30.0);
            }

            // The car is currently driving to a custom waypoint that was already determined
            // to be a safe target. Keep moving towards it
            if (this->car_->trackingNavWaypoint) {
                sdcAngle targetAngle = this->car_->AngleToTarget(this->car_->navWaypoint);
                this->car_->SetTargetDirection(targetAngle);

                if (this->car_->GetDistance(this->car_->navWaypoint) < 1) {
                    this->car_->trackingNavWaypoint = false;
                    this->car_->SetTurningLimit(10.0);
                }
            } else {
                // At this point, need to find a gap in the objects presented ahead of the car and
                // begin driving towards it
                double maxWidth = -1;
                double dist = 0;
                double prevDist = 0;
                sdcAngle targetAngle = this->car_->GetOrientation();

                // If there isn't an object directly in front of us, we can safely drive forward
                if (!this->car_->ObjectDirectlyAhead()) {
                    this->car_->navWaypoint = math::Vector2d(this->car_->x + cos(this->car_->GetOrientation().angle) * 4, this->car_->y + sin(this->car_->GetOrientation().angle) * 4);
                    this->car_->trackingNavWaypoint = true;
                    break;
                }

                // Loop through all objects in front of the car, find the space with the largest width
                // and store the point between them
                math::Vector2d prevPoint = math::Vector2d(this->car_->frontObjects[0].right.GetLateralDist() + FRONT_OBJECT_COLLISION_WIDTH + 0.2, this->car_->frontObjects[0].right.GetLongitudinalDist());
                // Angle closest to 0 that it's safe to drive through
                double bestMargin = 2 * PI;
                math::Vector2d curPoint;
                for (int i = 0; i < this->car_->frontObjects.size(); i++) {
                    curPoint = this->car_->frontObjects[i].right.GetAsPoint();
                    if (curPoint.Distance(prevPoint) > FRONT_OBJECT_COLLISION_WIDTH) {
                        // Point is on our left
                        if (curPoint.x < 0) {
                            math::Vector2d newPoint = math::Vector2d(prevPoint.x - FRONT_OBJECT_COLLISION_WIDTH/2., prevPoint.y);
                            sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                            if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                                bestMargin = newAngle.FindMargin(sdcAngle(0)).angle;
                                this->car_->navWaypoint = math::Vector2d(this->car_->x + cos((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->car_->y + sin((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                            }
                        }
                        // Point is on our right
                        else {
                            math::Vector2d newPoint = math::Vector2d(curPoint.x + FRONT_OBJECT_COLLISION_WIDTH/2., curPoint.y);
                            sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                            if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                                bestMargin = newAngle.FindMargin(sdcAngle(0)).angle;
                                this->car_->navWaypoint = math::Vector2d(this->car_->x + cos((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->car_->y + sin((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                            }
                        }
                    }
                    prevPoint = this->car_->frontObjects[i].left.GetAsPoint();
                }
                curPoint = math::Vector2d(prevPoint.x, 0);
                if (curPoint.Distance(prevPoint) > FRONT_OBJECT_COLLISION_WIDTH + 0.2) {
                    math::Vector2d newPoint = math::Vector2d(prevPoint.x - FRONT_OBJECT_COLLISION_WIDTH/2., prevPoint.y);
                    sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                    if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                        this->car_->navWaypoint = math::Vector2d(this->car_->x + cos((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->car_->y + sin((newAngle + this->car_->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                    }
                }

                this->car_->trackingNavWaypoint = true;
            }
            break;
        }

        case notAvoiding: // fall through
        default:
            this->currentState = this->DEFAULT_STATE;
            break;
    }

}

/*
 * Executes a turn at an intersection
 */
void sdcHLC::GridTurning(int turn) {
    int progress = this->car_->waypointProgress;
    if (turn == 3) {
        this->car_->waypointProgress++;
        this->currentState = STOP;
        return;
    } else if (turn == 0) {
        this->car_->waypointProgress++;
        this->car_->turning = false;
        return;
    }
    math::Vector2d nextTarget = {
        WAYPOINT_VEC[progress+1].pos.first,
        WAYPOINT_VEC[progress+1].pos.second
    };
    sdcAngle targetAngle = this->car_->AngleToTarget(nextTarget);
    this->car_->SetTargetDirection(targetAngle);
    sdcAngle margin = this->car_->GetOrientation().FindMargin(targetAngle);
    if (margin < .1 && margin > -.1) {
        this->car_->turning = false;
        this->car_->waypointProgress++;
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
    this->car_->SetTurningLimit(30.0);

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
    switch(this->currentPerpendicularState_) {
        // Done with perpendicular parking and sets car to stop state
        case donePark:
            this->llc_->StopReverse();
            this->llc_->Stop();
            this->car_->SetTurningLimit(10.0);
            this->car_->parkingSpotSet = false;
            this->currentState = STOP;
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
                this->currentPerpendicularState_ = backPark;
            } else {
                this->llc_->StopReverse();
                this->car_->SetTargetSpeed(0.5);
                sdcAngle margin = this->car_->GetOrientation().FindMargin(
                    this->car_->targetParkingAngle);
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
                        this->car_->parkingAngleSet = false;
                        this->currentPerpendicularState_ = straightPark;
                    }
                }
                this->car_->SetTargetDirection(this->car_->targetParkingAngle);
            }
            break;

        // Backs up into parking spot until the space behind the car is good; only runs if the car
        // is aligned with the parking spot
        case straightPark:
            this->llc_->Reverse();
            this->car_->SetTargetDirection(this->car_->targetParkingAngle);
            this->car_->SetTargetSpeed(0.5);
            if (backLidar[numBackRays / 2] < 0.5) {
                this->currentPerpendicularState_ = donePark;
            }
            break;

        // Temporary stop state for the car while parking to help avoid hitting anything
        case stopPark:
            this->llc_->Stop();
            this->currentPerpendicularState_ = frontPark;
            break;

        // Backs into the parking spot
        case backPark:
            // If the car is too close to anything on the back or sides, stop and fix it
            if (backLidar.size() != 0) {
                for (int i = 0; i < backRightBound.size(); i++) {
                    if (backRightBound[i] < 0.7) {
                        this->currentPerpendicularState_ = stopPark;
                    }
                }
                for (int j = 0; j < backMidBound.size(); j++) {
                    if (backMidBound[j] < 0.5) {
                        this->currentPerpendicularState_ = stopPark;
                    }
                }
                for (int k = 0; k < backLeftBound.size(); k++) {
                    if (backLeftBound[k] < 0.7) {
                        this->currentPerpendicularState_ = stopPark;
                    }
                }
            }
            if (leftBackSideLidar.size() != 0) {
                for (int l = 0; l < backLeftSideBound.size(); l++) {
                    if (leftBackSideLidar[l] < 0.25) {
                        this->currentPerpendicularState_ = stopPark;
                    }
                }
            }
            if (rightBackSideLidar.size() != 0) {
                for (int m = 0; m < backRightSideBound.size(); m++) {
                    if (rightBackSideLidar[m] < 0.25) {
                        this->currentPerpendicularState_ = stopPark;
                    }
                }
            }

            // Sets a target angle for the car for when it's done parking
            if (!this->car_->parkingAngleSet) {
                this->car_->targetParkingAngle = (this->car_->GetOrientation() - (3*PI)/2);
                this->car_->SetTargetDirection(2*PI - this->car_->targetParkingAngle);
                this->car_->parkingAngleSet = true;
                break;
            } else {
                this->car_->SetTargetDirection(2*PI - this->car_->targetParkingAngle);
                this->llc_->Reverse();
                this->car_->SetTargetSpeed(0.5);
            }

            // Check to see if current direction is the same as targetParkingAngle
            sdcAngle margin = this->car_->GetOrientation().FindMargin(
                this->car_->targetParkingAngle);

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
                    this->car_->parkingAngleSet = false;
                    this->currentPerpendicularState_ = straightPark;
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
    this->car_->SetTurningLimit(30.0);

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
    switch(this->currentParallelState_) {
        // Drive back while turning right into parking spot
        case rightBack:
            if (!this->car_->parkingAngleSet) {
                this->car_->targetParkingAngle = this->car_->GetOrientation();
                this->car_->parkingAngleSet = true;
            } else {
                // Turn wheels left after turning 45 degrees
                if (this->car_->GetOrientation() > this->car_->targetParkingAngle + PI/4) {
                    this->currentParallelState_ = leftBack;
                    break;
                }
                this->llc_->Reverse();
                this->car_->SetTargetDirection(this->car_->targetParkingAngle - PI/2);
                this->car_->SetTargetSpeed(0.35);
                break;
            }

        // Drive back while turning left
        case leftBack:
            if (backLidar.size() != 0 && frontLidar.size() != 0) {
                sdcAngle margin = this->car_->GetOrientation().FindMargin(
                    this->car_->targetParkingAngle);
                double spaceMargin = std::abs(
                    backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
                // If the car is aligned in the parking spot, begin driving forward
                if (margin < 0.01 &&  spaceMargin < 0.05) {
                    this->currentParallelState_ = straightForward;
                    break;
                }
            }

            // If the car gets too close to anything behind it, pull forward while turning right
            if (backLidar.size() != 0) {
                for (int i = 0; i < backRightBound.size(); i++) {
                    if (backRightBound[i] < 0.5) {
                        this->currentParallelState_ = rightForward;
                    }
                }
                for (int j = 0; j < backMidBound.size(); j++) {
                    if (backMidBound[j] < 0.3) {
                        this->currentParallelState_ = rightForward;
                    }
                }
                for (int k = 0; k < backLeftBound.size(); k++) {
                    if (backLeftBound[k] < 0.5) {
                        this->currentParallelState_ = rightForward;
                    }
                }
            }
            this->car_->SetTargetDirection(this->car_->targetParkingAngle + PI/2);
            this->llc_->Reverse();
            this->car_->SetTargetSpeed(0.35);
            break;

        // Drive forward while turning right
        case rightForward:
            if (backLidar.size() != 0 && frontLidar.size() != 0) {
                sdcAngle margin = this->car_->GetOrientation().FindMargin(
                    this->car_->targetParkingAngle);
                double spaceMargin = std::abs(
                    backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
                if (margin < 0.01 &&  spaceMargin < 0.05) {
                    this->currentParallelState_ = straightForward;
                    break;
                }
            }

            // Back up if the car gets too close to anything in front
            if (frontLidar.size() != 0) {
                for (int i = 0; i < frontRightBound.size(); i++) {
                    if (frontRightBound[i] < 0.9) {
                        this->currentParallelState_ = leftBack;
                    }
                }
                for (int j = 0; j < frontMidBound.size(); j++) {
                    if (frontMidBound[j] < 0.5) {
                        this->currentParallelState_ = leftBack;
                    }
                }
                for (int k = 0; k < frontLeftBound.size(); k++) {
                    if (frontLeftBound[k] < 0.9) {
                        this->currentParallelState_ = leftBack;
                    }
                }
            }
            this->llc_->StopReverse();
            this->car_->SetTargetDirection(this->car_->targetParkingAngle - PI/2);
            this->car_->SetTargetSpeed(0.35);
            break;

        // Pull forward while in parking spot, finishes parallel parking if the car is aligned in the spot
        // and the space in front and back are roughly the same
        case straightForward:
        {
            double frontSpace = frontLidar[numFrontRays/2];
            double backSpace = backLidar[numBackRays/2];
            if (frontSpace == backSpace) {
                this->currentParallelState_ = doneParallel;
                break;
            } else if (frontSpace > backSpace) {
                double spaceMargin = std::abs(frontSpace - backSpace);
                if (spaceMargin < 0.01) {
                    this->currentParallelState_ = doneParallel;
                    break;
                } else {
                    this->car_->SetTargetDirection(this->car_->targetParkingAngle);
                    this->llc_->StopReverse();
                    this->car_->SetTargetSpeed(0.35);
                }
            } else {
                double spaceMargin = std::abs(frontSpace - backSpace);
                if (spaceMargin < 0.01) {
                    this->currentParallelState_ = doneParallel;
                    break;
                } else {
                    this->car_->SetTargetDirection(this->car_->targetParkingAngle);
                    this->llc_->Reverse();
                    this->car_->SetTargetSpeed(0.35);
                }
            }
            break;
        }

        // Finished parallel parking and sets current state to stop state
        case doneParallel:
            this->llc_->Stop();
            this->llc_->StopReverse();
            this->car_->SetTurningLimit(10.0);
            this->currentState = STOP;
            break;
    }
}

//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcHLC::GenerateWaypoints() {
    this->car_->GetNSEW();
    initializeGraph();
    const int start = getFirstIntersection();
    int dest;
    for (int i = 0; i < intersections.size(); ++i) {
        if (intersections[i].waypoint.pos.first == destination.first
                && intersections[i].waypoint.pos.second == destination.second) {
            dest = i;
        }
    }
    std::vector<int> path;
    removeStartingEdge(start);
    path = dijkstras(start, dest);
    insertWaypointTypes(path, this->car_->currentDir);
    for (int i = path.size()-1; i >=0; --i) {
        WAYPOINT_VEC.push_back(intersections[path[i]].waypoint);
    }
}

std::vector<int> sdcHLC::dijkstras(int start, int dest) {
    std::vector<int> path;
    int current;
    intersections[start].dist = 0;
    intersections[start].previous = -1;
    double distance;

    // initializes the unvisited list by placing all of start's neighbors in it
    for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
        // push back each neighbor of the start into unvisited
        unvisited.push_back(intersections[start].neighbors_pairs[n].first);
        // set the distance of each neighbor to the distance of the edge
        // from start to neighbor and make neighbor previous = start
        intersections[intersections[start].neighbors_pairs[n].first].dist =
            intersections[start].neighbors_pairs[n].second;
        intersections[intersections[start].neighbors_pairs[n].first].previous =
            intersections[start].place;
    }

    // BFS using the unvisted FI FO vector, if unvisited is 0 then we have
    // visited all intersections
    while (unvisited.size() != 0) {
        current = unvisited[0];
        for (int n = 0; n < intersections[current].neighbors_pairs.size(); ++n) {
            // distance to the neighbor from current intersection
            distance = intersections[current].neighbors_pairs[n].second;
            // if the distance of the current intersection + the distance from
            // the current intersection to neighbor is smaller than the distance
            // to neighbor, update distance and previous
            if (intersections[intersections[current].neighbors_pairs[n].first].dist >
                    intersections[current].dist + distance) {
                // update distance
                intersections[intersections[current].neighbors_pairs[n].first].dist =
                    intersections[current].dist + distance;
                // update previous
                intersections[intersections[current].neighbors_pairs[n].first]
                    .previous = intersections[current].place;
            }
            // if the neighbor has not been visited then push back into unvisited
            if (intersections[intersections[current].neighbors_pairs[n].first].visited == 0) {
                // push back neighbor into unvisited
                unvisited.push_back(intersections[current].neighbors_pairs[n].first);
            }
            // mark the current intersection as visited
            intersections[current].visited = 1;
        }
        //pop front
        unvisited.erase(unvisited.begin());
    }

    //crawl backwards from dest to start to get the path
    for (int i = intersections[dest].place; i != -1;) {
        path.push_back(i);
        i = intersections[i].previous;
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

    //place the intersections into intersections
    intersections = { aa, ab, ac, ad, ae,
                      ba, bb, bc, bd, be,
                      ca, cb, cc, cd, ce,
                      da, db, dc, dd, de,
                      ea, eb, ec, ed, ee };
    //make the distance to all intersections infinity
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
}

int sdcHLC::getFirstIntersection() {
    std::pair<double,double> firstIntr;
    int firstIntersection;

    switch(this->car_->currentDir) {

        case west:
            firstIntr = {-1000,0};
            for (int i = 0; i < intersections.size();++i) {
                if (this->car_->y < intersections[i].waypoint.pos.second+5
                        && this->car_->y > intersections[i].waypoint.pos.second-5
                        && intersections[i].waypoint.pos.first < this->car_->x - 10
                        && intersections[i].waypoint.pos.first > firstIntr.first)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case east:
            firstIntr = {1000,0};
            for (int i = 0; i < intersections.size();++i) {
                if (this->car_->y < intersections[i].waypoint.pos.second+5
                        && this->car_->y > intersections[i].waypoint.pos.second-5
                        && intersections[i].waypoint.pos.first > this->car_->x + 10
                        && intersections[i].waypoint.pos.first < firstIntr.first) {
                    firstIntr = intersections[i].waypoint.pos;
                }
            }
            break;

        case north:
            firstIntr = {0,1000};
            for (int i = 0; i < intersections.size();++i) {
                if (this->car_->x < intersections[i].waypoint.pos.first+5
                        && this->car_->x > intersections[i].waypoint.pos.first-5
                        && intersections[i].waypoint.pos.second > this->car_->y + 10
                        && intersections[i].waypoint.pos.second < firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case south:
            firstIntr = {0,-1000};
            for (int i = 0; i < intersections.size();++i) {
                if (this->car_->x < intersections[i].waypoint.pos.first+5
                        && this->car_->x > intersections[i].waypoint.pos.first-5
                        && intersections[i].waypoint.pos.second < this->car_->y - 10
                        && intersections[i].waypoint.pos.second > firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;
    }

    for (int i = 0; i < intersections.size();i++) {
        if (firstIntr.first == intersections[i].waypoint.pos.first
                && firstIntr.second == intersections[i].waypoint.pos.second) {
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
                    intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
                    break;
                case east:
                    intersections[current].waypoint.waypointType = WaypointType_TurnRight;
                    break;
                case west:
                    intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
                case south:
                    break;
            }
            break;
        case south:
            switch (nextDir) {
                case south:
                    intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
                    break;
                case east:
                    intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
                    break;
                case west:
                    intersections[current].waypoint.waypointType = WaypointType_TurnRight;
                case north:
                    break;
            }
            break;
        case east:
            switch (nextDir) {
                case north:
                    intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
                    break;
                case south:
                    intersections[current].waypoint.waypointType = WaypointType_TurnRight;
                    break;
                case east:
                    intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
                case west:
                    break;
            }
            break;
        case west:
            switch (nextDir) {
                case north:
                    intersections[current].waypoint.waypointType = WaypointType_TurnRight;
                    break;
                case south:
                    intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
                    break;
                case west:
                    intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
                case east:
                    break;
            }
            break;
    }
    curDir = nextDir;
  }
  intersections[path[0]].waypoint.waypointType = WaypointType_Stop;
}

void sdcHLC::removeStartingEdge(int start) {
    Direction dir = east;
    switch (dir) {
        case north:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start - 1) {
                    intersections[start].neighbors_pairs[n].second =
                        std::numeric_limits<double>::infinity();
                }
            }
            break;
        case south:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start + 1) {
                    intersections[start].neighbors_pairs[n].second =
                        std::numeric_limits<double>::infinity();
                }
            }
            break;
        case east:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start - size) {
                    intersections[start].neighbors_pairs[n].second =
                        std::numeric_limits<double>::infinity();
                }
            }
            break;
        case west:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start + size) {
                    intersections[start].neighbors_pairs[n].second =
                        std::numeric_limits<double>::infinity();
                }
            }
            break;
    }
}
