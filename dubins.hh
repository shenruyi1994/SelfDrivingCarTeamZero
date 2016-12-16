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

typedef struct {
  double seg1;
  double seg2;
  double seg3;
  double length;
  std::string type;
  
} Path;

namespace gazebo {
    class GAZEBO_VISIBLE dubins {

  public:
    dubins();
    ~dubins() {}

    int calculateDubins(Waypoints* waypoints);


  private:
    //Control* control_;

  };
}

#endif
