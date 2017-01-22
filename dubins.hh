#ifndef __DUBINS_HH__
#define __DUBINS_HH__

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include "globals.hh"
#include "Waypoints.hh"
//#include <string>
#include "sdcLLC.hh"
#include "dubins.hh"
#include <opencv2/opencv.hpp>

enum PathDirection {
  DUBINS_LEFT = -1,
  DUBINS_STRAIGHT = 0,
  DUBINS_RIGHT = 1
};

//Stores all the necessary information for a dubins path
typedef struct {
  double seg1;
  double seg2;
  double seg3;
  double length;
  double rotationAngle;
  PathDirection* dirs;
  cv::Point3d origin;
} Path;



//Control where first is steering direction(min = -1, straight = 0, max = 1), and second # of timesteps to apply steering direction
typedef struct{
  int direction;
  double distance;
} Control;

class GAZEBO_VISIBLE dubins {

  public:
    dubins();
    ~dubins() {}

    Path calculateDubins(std::vector<Waypoint>, Waypoint);
    std::vector<Control> pathToControls(Path);
    cv::Point3d leftTurn(double, double, double, double);
    cv::Point3d rightTurn(double, double, double, double);
    cv::Point3d straightTurn(double, double, double, double);

  private:
    //Control* control_;

  };

#endif
