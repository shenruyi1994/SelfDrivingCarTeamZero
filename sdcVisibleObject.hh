#ifndef _sdcVisibleObject_hh_
#define _sdcVisibleObject_hh_

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"

namespace gazebo {
  class sdcVisibleObject {
  public:
    sdcLidarRay left_;
    sdcLidarRay right_;
    double dist_;
    double lineSlope_;
    double lineIntercept_;

    sdcVisibleObject();
    sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist);

    bool IsSameObject(sdcVisibleObject other);
    math::Vector2d EstimateUpdate();
    math::Vector2d GetProjectedPosition(int numSteps);
    math::Vector2d GetProjectedPositionAtTime(double time); // TODO: implement
    void Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist);
    void Update(sdcVisibleObject newObject);

    void SetTracking(bool isTracking);
    bool IsTracking();
    math::Vector2d GetCenterPoint();
    double GetEstimatedSpeed();
    double GetEstimatedYSpeed();
    double GetEstimatedXSpeed();

    math::Vector2d FitLineToPoints(std::vector<math::Vector2d> points, math::Vector2d newPoint);

  private:
    static const double UNCERTAINTY_RATIO;

    math::Vector2d centerpoint_;

    std::vector<math::Vector2d> prevPoints_;

    double estimatedXSpeed_;
    double estimatedYSpeed_;

    double confidence_;

    bool tracking_;
    bool brandSpankinNew_;

    math::Vector2d GetCenterPoint(sdcLidarRay left, sdcLidarRay right, double dist);
  };
}
#endif
