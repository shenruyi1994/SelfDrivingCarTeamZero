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

#ifndef _sdcCar_hh_
#define _sdcCar_hh_

#include <string>
#include <vector>
#include <exception>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

#include "globals.hh"

#include "sdcAngle.hh"
#include "sdcIntersection.hh"
#include "sdcSensorData.hh"
#include "sdcWaypoint.hh"
#include <opencv2/opencv.hpp>


namespace gazebo {
  class sdcHLC;
  class GAZEBO_VISIBLE sdcCar : public ModelPlugin {
    friend class sdcLLC;
    friend class sdcHLC;

  public:
    // Constructor for sdcCar
    sdcCar();
    ~sdcCar();

    // These methods are called by Gazebo during the loading and initializing
    // stages of world building and populating
    virtual void Load(physics::ModelPtr _model_, sdf::ElementPtr _sdf);
    virtual void Init();

  private:
    // Bound to Gazebo's world update, gets called every tick of the simulation
    void OnUpdate();

    // Holds the bound connection to Gazebo's update, necessary in order to properly
    // receive updates
    std::vector<event::ConnectionPtr> connections_;

    // The Gazebo model_ representation of the car
    physics::ModelPtr model_;
    // Contains the wheel joints_ that get operated on each tick for movement
    std::vector<physics::JointPtr> joints_;
    // A link to the chassis_ of the car, mainly used for access to physics variables
    // related to the car's state
    physics::LinkPtr chassis_;

    // The velocity_ of the car
    math::Vector3 velocity_;

    // These variables are mostly set in the SDF for the car and relate to the
    // physical parameters of the vehicle
    double frontPower_, rearPower_;
    double maxSpeed_;
    double wheelRadius_;

    double steeringRatio_;
    double tireAngleRange_;

    double aeroLoad_;
    double swayForce_;

    //////////////////////////////////////////
    // Begin Non-Gazebo Related Definitions //
    //////////////////////////////////////////

    enum RelativeDirection { forward, aligned, backward, right, left };

    ///////////////////////////
    // SDC-defined variables //
    ///////////////////////////

    RelativeDirection destDir_;
    RelativeDirection destDirSide_;
    Direction currentDir_;

    // High and low level controllers for the car
    sdcHLC* hlc_;

    double gas_;   // variable that accelerates the car
    double brake_; // variable that brake_s the car

    // Scalars for accelrating and braking
    double accelRate_;
    double brakeRate_;

    // Position/rotation variables
    sdcAngle yaw_;

    // Waypoint variables
    int waypointProgress_;

    // Intersection variables
    bool stoppedAtSign_;
    int ignoreStopSignsCounter_;
    int atIntersection_;

    // Car limit variables
    int maxCarSpeed_;
    double maxCarReverseSpeed_;
    double turningLimit_;

    // Flags for the car's actions
    bool turning_;
    bool reversing_;
    bool stopping_;

    // Movement parameters
    sdcAngle targetDirection_;
    double targetSteeringAmount_;
    double steeringAmount_;
    double targetSpeed_;

    // Parking variables
    sdcAngle targetParkingAngle_;
    bool parkingAngleSet_;
    bool isFixingParking_;
    bool parkingSpotSet_;

    // Follow variables
    bool isTrackingObject_;
    int stationaryCount_;

    // Avodiance variables
    math::Vector2d navWaypoint_;
    bool trackingNavWaypoint_;

    // Variables relating to tracking objects in front of the car
    std::vector<sdcVisibleObject*> frontObjects_;
    int frontLidarLastUpdate_;

    // The x and y position of the car
    double x_;
    double y_;

    double width_;
    double length_;

    /////////////////////////
    // SDC-defined methods //
    /////////////////////////

    // Helper methods
    void FrontLidarUpdate();
    void UpdateFrontObjects(std::vector<sdcVisibleObject*> newObjects);

    sdcAngle AngleToTarget(math::Vector2d target) const;
    bool ObjectDirectlyAhead() const;
    bool IsObjectDirectlyAhead(const sdcVisibleObject* obj) const;
    bool ObjectOnCollisionCourse() const;
    bool IsObjectOnCollisionCourse(const sdcVisibleObject* obj) const;
    bool IsObjectTooFast(const sdcVisibleObject* obj) const;
    bool IsObjectTooFurious(const sdcVisibleObject* obj) const;

    bool IsMovingForwards() const;
    double GetSpeed() const;
    double GetDistance(math::Vector2d navWaypoint_) const;
    sdcAngle GetDirection() const;
    sdcAngle GetOrientation() const;
    void GetNSEW();
    double GetMaxSafeTime() const;
    double GetMinTurningRadius() const;

    void SetTargetDirection(sdcAngle direction);
    void SetTargetPoint(cv::Point2d targetPoint);
    void SetTargetSteeringAmount(double a);
    void SetSteeringAmount(double a);
    void SetTargetSpeed(double s);
    void SetTurningLimit(double limit);

    void SetAccelRate(double rate = 1.0);
    void SetBrakeRate(double rate = 1.0);
  };
}
#endif
