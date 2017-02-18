#ifndef __DUBINS_HH__
#define __DUBINS_HH__

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <opencv2/opencv.hpp>


#include "globals.hh"
#include "Waypoints.hh"

enum PathDirection {
  DUBINS_LEFT = -1,
  DUBINS_STRAIGHT = 0,
  DUBINS_RIGHT = 1
};

//Stores all the necessary information for a dubins path
struct Path {
  double seg1;
  double seg2;
  double seg3;
  double length;

  PathDirection dir1;
  PathDirection dir2;
  PathDirection dir3;
  cv::Point3d origin;

  double rotationAngle;

  Path& operator *=(double n) {
    this->seg1 *= n;
    this->seg2 *= n;
    this->seg3 *= n;
    this->length *= n;
    return *this;
  }
};

//Control where first is steering direction(min = -1, straight = 0, max = 1), and second # of timesteps to apply steering direction
typedef struct{
  int direction;
  double distance;
} Control;

class GAZEBO_VISIBLE dubins {

  public:
    dubins();
    ~dubins() {}

    Path calculateDubins(Waypoint waypoint, Waypoint carPoint, double minRadius);
    std::vector<Control> pathToControls(Path);
    cv::Point3d leftTurn(double, double, double, double);
    cv::Point3d rightTurn(double, double, double, double);
    cv::Point3d straightTurn(double, double, double, double);

  private:
    double scalingFactor_ = 1;
  };

#endif
