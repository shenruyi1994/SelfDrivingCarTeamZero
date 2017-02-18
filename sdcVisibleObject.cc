/*
 * This class provides a wrapper for objects we see, created by two lidar rays and
 * a distance. This class provides methods that return information about these
 * objects, including estimates about future position based upon past data.
 */

#include "sdcVisibleObject.hh"

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"
#include "sdcUtils.hh"

using namespace gazebo;

// How far off our estimates can be before we assume we're seeing a differnt object
const double sdcVisibleObject::UNCERTAINTY_RATIO = 0.3;
const double PI = 3.14159265359;

/*
 * Default constructor that leaves all parameters at default values
 */
sdcVisibleObject::sdcVisibleObject() {
  brightness_ = 0;
}

/*
 * Visible objects are obstructions detected by Lidar rays. They have estimated
 * movement parameters in order to help us predict their motion and track them
 * across multiple sensor readings.
 */
sdcVisibleObject::sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist, int leftRayIndex, int rightRayIndex) {
  left_ = left;
  right_ = right;
  dist_ = dist;

  centerpoint_ = GetCenterPoint();
  prevPoints_.push_back(centerpoint_);

  estimatedXSpeed_ = 0;
  estimatedYSpeed_ = 0;
  confidence_ = 0.01;

  tracking_ = false;
  brandSpankinNew_ = true;
  brightnessDetected = false;
    
  leftRayIndex_ = leftRayIndex;
  rightRayIndex_ = rightRayIndex;
    
}

/*
 * Returns true if the given object is a possible new position of this object
 */
bool sdcVisibleObject::IsSameObject(sdcVisibleObject* other) const {
  double new_left_edge = this->getLeftRay().GetLateralDist();
  double new_right_edge = this->getRightRay().GetLateralDist();
  
  double old_left_edge = other->getLeftRay().GetLateralDist();
  double old_right_edge = other->getRightRay().GetLateralDist();

  double uncertainty = fabs(new_left_edge - old_left_edge) + fabs(new_right_edge - old_right_edge);
  
  std::cout << "uncertainty: " << uncertainty << std::endl;
  return uncertainty < UNCERTAINTY_RATIO;
}

/*
 * Returns the estimated total speed of this object relative to the car's speed
 */
double sdcVisibleObject::GetEstimatedSpeed() const {
  return pythag_thm(estimatedXSpeed_, estimatedYSpeed_);
}

/*
 * Returns the current estimated speed parallel to the car's orientation (towards or away
 * from the car if the object is ahead of the car), and relative to the car's speed
 *
 * Negative speeds are moving towards the car, positive away
 */
double sdcVisibleObject::GetEstimatedYSpeed() const {
  return estimatedYSpeed_;
}

/*
 * Returns the current estimated speed perpendicular to the car's orientation (across the
 * car's field of view), and realtive to the car's speed
 *
 * Negative speeds are moving left, positive right
 */
double sdcVisibleObject::GetEstimatedXSpeed() const {
  return estimatedXSpeed_;
}

/*
 * Calculates an estimated new position this object would be at with it's given estimated
 * speed and direction
 */
math::Vector2d sdcVisibleObject::EstimateUpdate() const {
  double newX = centerpoint_.x + estimatedXSpeed_;
  double newY = centerpoint_.y + estimatedYSpeed_;
  return math::Vector2d(newX, newY);
}

/*
 * Method to calculate the projected position some numSteps into the future.
 *
 * Currently unused.
 */
math::Vector2d sdcVisibleObject::GetProjectedPosition(int numSteps) const {
  double newX = centerpoint_.x + estimatedXSpeed_ * numSteps;
  double newY = centerpoint_.y + estimatedYSpeed_ * numSteps;
  return math::Vector2d(newX, newY);
}

/*
 * TODO: implement this, figure out what time means
 */
math::Vector2d sdcVisibleObject::GetProjectedPositionAtTime(double time) const {
  return math::Vector2d(0, 0);
}

/*
 * Given new readings for the location of this object, update the stored parameters
 * to learn it's projected speed and direction
 */
void sdcVisibleObject::Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist) {
  confidence_ = fmin(1.0, confidence_ + 0.01);

  // Get the centerpoint of the new information
  math::Vector2d newCenterpoint = GetCenterPoint(newLeft, newRight, newDist);

  // Calculate the speed moving from the current point to the new point
  double newEstimatedXSpeed = (newCenterpoint.x - centerpoint_.x);
  double newEstimatedYSpeed = (newCenterpoint.y - centerpoint_.y);

  // If this object has already been updated at least once, try to learn the speed
  // over time
  if (!brandSpankinNew_) {
    double alpha = fmax((newDist * .005), (.1 - newDist * .005));
    newEstimatedXSpeed = (alpha * newEstimatedXSpeed) + ((1 - alpha) * estimatedXSpeed_);
    newEstimatedYSpeed = (alpha * newEstimatedYSpeed) + ((1 - alpha) * estimatedYSpeed_);
  }

  // Update the estimates for this object's speed
  estimatedXSpeed_ = newEstimatedXSpeed;
  estimatedYSpeed_ = newEstimatedYSpeed;

  // Fit a line to the points this object has been at, and store that line's information
  if (prevPoints_.size() > 0) {
    math::Vector2d newLineCoefficients = FitLineToPoints(prevPoints_, newCenterpoint);
    lineSlope_ = newLineCoefficients.x;
    lineIntercept_ = newLineCoefficients.y;
  }

  // Maintain prevPoints to never be larger than 16 to help ensure accurate information
  if (prevPoints_.size() > 15) {
    prevPoints_.erase(prevPoints_.begin());
  }
  prevPoints_.push_back(newCenterpoint);

  // Update the object's information
  centerpoint_ = newCenterpoint;

  left_ = newLeft;
  right_ = newRight;
  dist_ = newDist;

  // This object has now been updated, so set this flag accordingly
  brandSpankinNew_ = false;
}

/*
 * This method takes in a vector of multiple points and attemps to fit a line to these points.
 * This allows us to project its path and determine whether or not there is a chance for
 * the object to hit us. Method returns the predicted slope and Y-intercept based upon the
 * vector of points.
 */
math::Vector2d sdcVisibleObject::FitLineToPoints(
    std::vector<math::Vector2d> points, math::Vector2d newPoint) const {
  int numPoints = points.size();

  // Calculate several necessary sums over all points
  double sumX=0, sumY=0, sumXY=0, sumX2=0;
  for (int i=0; i<numPoints; i++) {
    sumX += points[i].x;
    sumY += points[i].y;
    sumXY += points[i].x * points[i].y;
    sumX2 += points[i].x * points[i].x;
  }

  sumX += newPoint.x;
  sumY += newPoint.y;
  sumXY += newPoint.x * newPoint.y;
  sumX2 += newPoint.x * newPoint.x;

  // Get the averages for x and y
  double xMean = sumX / (numPoints + 1);
  double yMean = sumY / (numPoints + 1);

  // Calculate the denominator for the slope calculation
  double denom = sumX2 - sumX * xMean;

  // Calculate the slope and intercept of the line
  double slope = (sumXY - sumX * yMean) / denom;
  double yInt = yMean - slope * xMean;

  return math::Vector2d(slope, yInt);
}

/*
 * Update this object with the given object's parameters
 */
void sdcVisibleObject::Update(sdcVisibleObject* newObject) {
  Update(newObject->left_, newObject->right_, newObject->dist_);
}

/*
 * Set whether we are tracking this object
 */
void sdcVisibleObject::SetTracking(bool isTracking) {
  tracking_ = isTracking;
}

/*
 * Get whetehr this object is being tracked
 */
bool sdcVisibleObject::IsTracking() const {
  return tracking_;
}

/*
 * Gets the centerpoint of this object based on the left and right rays
 */
math::Vector2d sdcVisibleObject::GetCenterPoint() const {
  return GetCenterPoint(left_, right_, dist_);
}

/*
 * Gets the centerpoint of the two given rays in (x,y) coordinates
 */
math::Vector2d sdcVisibleObject::GetCenterPoint(sdcLidarRay left,
                                                sdcLidarRay right,
                                                double dist) const {
  sdcLidarRay midRay = sdcLidarRay(left.angle.GetMidAngle(right.angle), dist);
  double x = midRay.GetLateralDist();
  double y = midRay.GetLongitudinalDist();

  // double x = (left.GetLateralDist() + right.GetLateralDist()) / 2.;
  // double y = (left.GetLongitudinalDist() + right.GetLongitudinalDist()) / 2.;
  return math::Vector2d(x, y);
}

sdcLidarRay sdcVisibleObject::getLeftRay() const {
  return left_;
}

sdcLidarRay sdcVisibleObject::getRightRay() const {
  return right_;
}

int sdcVisibleObject::getLeftRayIndex() const{
    return leftRayIndex_;
}

int sdcVisibleObject::getRightRayIndex() const{
    return rightRayIndex_;
}

void sdcVisibleObject::setBrightnessDetected(){
    brightnessDetected = true;
}

bool sdcVisibleObject::getBrightnessDetected() const{
    return brightnessDetected;
}