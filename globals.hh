#ifndef __CONSTANTS_HH__
#define __CONSTANTS_HH__

#include <vector>

#include "sdcAngle.hh"
#include "sdcIntersection.hh"
#include "sdcWaypoint.hh"

// SDC-defined constants
const double PI = 3.14159265359;

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

// How fast the car turns each update
const double STEERING_ADJUSTMENT_RATE = 0.02;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

// The width of the channel in front of the car for which we count objects as
// being directly in front of the car
const double FRONT_OBJECT_COLLISION_WIDTH = CAR_WIDTH + 0.5;

enum Direction { north, south, east, west };

#endif
