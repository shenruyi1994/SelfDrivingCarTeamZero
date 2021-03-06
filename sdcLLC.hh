#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

#include <array>
#include <deque>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <opencv2/opencv.hpp>

#include "globals.hh"
#include "sdcAngle.hh"
#include "Waypoints.hh"
#include "dubins.hh"


//placeholder structs for now
namespace gazebo {
  class sdcCar;
  class GAZEBO_VISIBLE sdcLLC {
  public:
    sdcLLC(sdcCar* car);
    ~sdcLLC() {
      delete dubins_;
    }

    void update();

    std::pair<Path, double> GetPathFromDistance(double distance) const;
    double GetDubinsAngle(double distance, bool genNew = true);
    cv::Point2d GetDubinsPoint(double distance, bool genNew = true);
    void GenerateNewDubins();
    bool BeyondPath(double distance) const;

    // Control methods
    void Accelerate(double amt = 1, double rate = 1.0);
    void Brake(double amt = 1, double rate = 1.0);
    void Stop();
    void Reverse();
    void StopReverse();
    sdcCar* car_;
    dubins* dubins_;
    std::vector<Path> paths_;
  private:
    std::array<Waypoint, 3> safeWaypoints_;
    std::array<Waypoint, 3> lastWaypoints_;
    std::array<std::deque<Waypoint>, 3> recentWaypoints_;
  };
}
#endif
