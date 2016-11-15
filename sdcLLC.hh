#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>




#include "globals.hh"
#include "Waypoints.hh"

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::FT               Number_type;
typedef CGAL::Arr_circle_segment_traits_2<Kernel> Traits;
typedef Traits::CoordNT CoordNT;
typedef Traits::Point_2 Point;
typedef Traits::Curve_2 Curve;
typedef Traits::Rational_point_2 Rational_point;
typedef Traits::Rational_segment_2 Segment;
typedef Traits::Rational_circle_2 Circle;
typedef CGAL::Arrangement_2<Traits> Arrangement;

//placeholder structs for now
typedef struct {
  int angle;
} SteeringAngle;

typedef struct {
  int timeStep;
} TimeStep;


typedef struct {
  std::pair<SteeringAngle, TimeStep> control;
} Control;


namespace gazebo {
  class sdcCar;
  class GAZEBO_VISIBLE sdcLLC {
  public:
    sdcLLC(sdcCar* car);
    ~sdcLLC() {}

    void update();
    void setWaypoints(Waypoints* waypoints);
    std::pair<SteeringAngle, TimeStep> calculateDubins(Waypoints* waypoints);

    // Control methods
    void Accelerate(double amt = 1, double rate = 1.0);
    void Brake(double amt = 1, double rate = 1.0);
    void Stop();
    void Reverse();
    void StopReverse();

  private:
    sdcCar* car_;
    Waypoints* waypoints_;
  };
}
#endif
