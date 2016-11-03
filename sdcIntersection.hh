#ifndef SDCINTERSECTION_HH_
#define SDCINTERSECTION_HH_

#include <vector>
#include <iostream>
#include <limits>

#include "sdcAngle.hh"
#include "sdcWaypoint.hh"
#include "globals.hh"

enum WaypointType {
    WaypointType_DriveStraight,
    WaypointType_TurnLeft,
    WaypointType_TurnRight,
    WaypointType_Stop
};

class sdcIntersection {
    public:
    // neighbor = pair.first
    // dist = pair.second
    std::vector<std::pair<int, double>> neighbors_pairs;
    int place;
    int previous;
    double dist = std::numeric_limits<double>::infinity();
    bool visited = 0;
    WaypointType wpType;
    sdcWaypoint waypoint;
};

#endif
