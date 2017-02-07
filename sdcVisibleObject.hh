#ifndef _sdcVisibleObject_hh_
#define _sdcVisibleObject_hh_

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"

namespace gazebo {
  class sdcVisibleObject {
  public:
    sdcVisibleObject();
    sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist);

    bool IsSameObject(sdcVisibleObject* other) const;
    math::Vector2d EstimateUpdate() const;
    math::Vector2d GetProjectedPosition(int numSteps) const;
    math::Vector2d GetProjectedPositionAtTime(double time) const; // TODO: implement
    void Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist);
    void Update(sdcVisibleObject* newObject);

    void SetTracking(bool isTracking);
    bool IsTracking() const;
    math::Vector2d GetCenterPoint() const;
    double GetEstimatedSpeed() const;
    double GetEstimatedYSpeed() const;
    double GetEstimatedXSpeed() const;

    sdcLidarRay Left() const { return left_; };
    sdcLidarRay Right() const { return right_; };
    double Dist() const { return dist_; };
    double LineSlope() const { return lineSlope_; };
    double LineIntercept() const { return lineIntercept_; };

    math::Vector2d FitLineToPoints(std::vector<math::Vector2d> points,
                                   math::Vector2d newPoint) const;

    sdcLidarRay getLeftRay() const;
    sdcLidarRay getRightRay() const;

  private:
    sdcLidarRay left_;
    sdcLidarRay right_;
    double dist_;
    double lineSlope_;
    double lineIntercept_;

    static const double UNCERTAINTY_RATIO;

    math::Vector2d centerpoint_;

    std::vector<math::Vector2d> prevPoints_;

    double estimatedXSpeed_;
    double estimatedYSpeed_;

    double confidence_;

    bool tracking_;
    bool brandSpankinNew_;

    math::Vector2d GetCenterPoint(sdcLidarRay left, sdcLidarRay right,
                                  double dist) const;
  };
}
#endif
