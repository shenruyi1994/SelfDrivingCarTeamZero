#ifndef __SDCHLC_HH__
#define __SDCHLC_HH__

#include <math.h>
#include <vector>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <opencv2/opencv.hpp>

#include "globals.hh"
#include "sdcIntersection.hh"
#include "sdcVisibleObject.hh"
#include "Waypoints.hh"
#include "CameraPlugin.hh"

namespace gazebo {
  class sdcCar;
  class sdcLLC;

  class GAZEBO_VISIBLE sdcHLC {
  public:
    sdcHLC(sdcCar* car);
    ~sdcHLC();

    void update();

    // The 'Brain' Methods
    void Drive();
    void MatchTargetDirection();
    void MatchTargetSpeed();

    // Dubins path following functions
    void FollowWaypoints();
    void UpdatePathDistance();
    cv::Point2d FindDubinsTargetPoint();
    sdcAngle CalculateTurningAngle(const math::Vector2d& point) const;
    double ScaledLookaheadDistance() const;
    void AngleWheelsTowardsTarget(const math::Vector2d& target);

    std::vector<double> steeringAngles;

    // Driving algorithms
    void AvoidObstacle();

    // Collision detection/avoidance functions
    void CheckNearbyObjectsForCollision();
    void DecideAvoidanceStrategy(const sdcVisibleObject* obj);
    bool CanStopBeforeObject(const sdcVisibleObject* obj) const;
    bool IsObjectOnCollisionCourse(const sdcVisibleObject* obj) const;
    bool DoMaximumBoundingBoxesCollide(const sdcVisibleObject* obj) const;
    double DoMaximumRadiiCollide(const sdcVisibleObject* obj) const;
    bool DoMaximumRadiiCollideAtTime(const sdcVisibleObject* obj,
                                     double time) const;
    bool DoAccurateVehicleShapesCollide(const sdcVisibleObject* obj,
                                        double time) const;
    bool DoAccurateVehicleShapesCollideAtTime(const sdcVisibleObject* obj,
                                              double time) const;
    std::vector<sdcWaypoint*>* ComputeAvoidancePath(sdcVisibleObject* obj,
                                                    math::Vector2d collision);
    cv::Point2d GetPositionAtTime(double time) const;
    sdcAngle GetAngleAtTime(double time) const;
    sdcAngle GetCollisionAngleAtTime(const sdcVisibleObject* obj,
                                     double time) const;
    void BackToLane();
    bool IsBackToLane();
    
  private:
    sdcCar* car_;
    sdcLLC* llc_;

    double pathDist_ = 0;
    common::Time lastTime_ = 0;
    double lastSpeed_ = 0;

    double lookaheadMin_ = 0.5;
    double lookaheadMax_ = 3.5;
    double lookaheadScalor_ = 0.6;

    double lastX_ = 0;
    double lastY_ = 0;
    
    common::Time lastUpdateTime_;

    // ================================================
    // 2016 states
    // ================================================
    enum MetaState {
      START_16, FINISH_16, ROAD_16, INTERSECTION_16, PARKING_16
    };

    // The different sub-states within the ROAD metastate
    enum RoadState {
      FOLLOW_16, // follow waypoints
      APPROACH_16, // avoidance maneuvers
      STOP_16, // emergency stop behind an obstacle
      WAIT_16, // wait behind a stopped car
      PASS_16, // pass a stopped car
      AVOID_16, // avoidance maneuvers, or going around an obstacles
      RETURN_16 // return to our lane after passing
    };

    // Current state of the car, 2016 version
    MetaState metaState_;
    RoadState roadState_;

    sdcVisibleObject* dangerousObj_;
  };
}
#endif
