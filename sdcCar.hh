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
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

    private:
        // Bound to Gazebo's world update, gets called every tick of the simulation
        void OnUpdate();

        // Holds the bound connection to Gazebo's update, necessary in order to properly
        // receive updates
        std::vector<event::ConnectionPtr> connections;

        // The Gazebo model representation of the car
        physics::ModelPtr model;
        // Contains the wheel joints that get operated on each tick for movement
        std::vector<physics::JointPtr> joints;
        // A link to the chassis of the car, mainly used for access to physics variables
        // related to the car's state
        physics::LinkPtr chassis;

        // The velocity of the car
        math::Vector3 velocity;

        // These variables are mostly set in the SDF for the car and relate to the
        // physical parameters of the vehicle
        double frontPower, rearPower;
        double maxSpeed;
        double wheelRadius;

        double steeringRatio;
        double tireAngleRange;

        double aeroLoad;
        double swayForce;

        //////////////////////////////////////////
        // Begin Non-Gazebo Related Definitions //
        //////////////////////////////////////////

        enum RelativeDirection { forward, aligned, backward, right, left };

        ///////////////////////////
        // SDC-defined variables //
        ///////////////////////////

        RelativeDirection destDir;
        RelativeDirection destDirSide;
        Direction currentDir;

        // High and low level controllers for the car
        sdcHLC* hlc_;

        double gas;   // variable that accelerates the car
        double brake; // variable that brakes the car

        // Scalars for accelrating and braking
        double accelRate;
        double brakeRate;

        // Position/rotation variables
        sdcAngle yaw;

        // Waypoint variables
        int waypointProgress;

        // Intersection variables
        bool stoppedAtSign;
        int ignoreStopSignsCounter;
        int atIntersection;

        // Car limit variables
        int maxCarSpeed;
        double maxCarReverseSpeed;
        double turningLimit;

        // Flags for the car's actions
        bool turning;
        bool reversing;
        bool stopping;

        // Movement parameters
        sdcAngle targetDirection;
        double targetSteeringAmount;
        double steeringAmount;
        double targetSpeed;

        // Parking variables
        sdcAngle targetParkingAngle;
        bool parkingAngleSet;
        bool isFixingParking;
        bool parkingSpotSet;

        // Follow variables
        bool isTrackingObject;
        int stationaryCount;

        // Avodiance variables
        math::Vector2d navWaypoint;
        bool trackingNavWaypoint;

        // Variables relating to tracking objects in front of the car
        std::vector<sdcVisibleObject> frontObjects;
        int frontLidarLastUpdate;

        // The x and y position of the car
        double x;
        double y;

        /////////////////////////
        // SDC-defined methods //
        /////////////////////////

        // Helper methods
        void FrontLidarUpdate();
        void UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects);

        sdcAngle AngleToTarget(math::Vector2d target);
        bool ObjectDirectlyAhead();
        bool IsObjectDirectlyAhead(sdcVisibleObject obj);
        bool ObjectOnCollisionCourse();
        bool IsObjectOnCollisionCourse(sdcVisibleObject obj);
        bool IsObjectTooFast(sdcVisibleObject obj);
        bool IsObjectTooFurious(sdcVisibleObject obj);

        bool IsMovingForwards();
        double GetSpeed();
        double GetDistance(math::Vector2d navWaypoint);
        sdcAngle GetDirection();
        sdcAngle GetOrientation();
        void GetNSEW();

        void SetTargetDirection(sdcAngle direction);
        void SetTargetSteeringAmount(double a);
        void SetTargetSpeed(double s);
        void SetTurningLimit(double limit);

        void SetAccelRate(double rate = 1.0);
        void SetBrakeRate(double rate = 1.0);
    };
}
#endif
