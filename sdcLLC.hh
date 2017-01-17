#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include "globals.hh"
#include "Waypoints.hh"
#include "dubins.hh"

#include <opencv2/opencv.hpp>




//placeholder structs for now
namespace gazebo {
  class sdcCar;
  class GAZEBO_VISIBLE sdcLLC {
  public:
    sdcLLC(sdcCar* car);
    ~sdcLLC() {}

    void update();

    cv::Point2d GetDubinsPoint(double distance) const;
    // Control methods
    void Accelerate(double amt = 1, double rate = 1.0);
    void Brake(double amt = 1, double rate = 1.0);
    void Stop();
    void Reverse();
    void StopReverse();
    sdcCar* car_;
    dubins* dubins_;
    Path path_;
  private:

  };
}
#endif
