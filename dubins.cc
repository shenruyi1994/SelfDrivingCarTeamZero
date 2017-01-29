#include "dubins.hh"

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

#include "globals.hh"
#include "sdcAngle.hh"
#include "sdcUtils.hh"
#include "Waypoints.hh"

using namespace gazebo;

dubins::dubins() {
  scalingFactor_ = MIN_TURNING_RADIUS;
}

//True mod function that does not return negative values
double mod(double a, double b) {
  double ret = fmod(a,b);
  if (ret < 0) { ret += b; }
  return ret;
}

/*
 * Calculates a dubins path consisting of a left turn, then a straight segment,
 * then a right turn.
 */
Path lsl(double initD, double finalD, double dist) {
  double t = mod((-initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)))),2*PI);
  double p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));
  double q = mod(finalD - atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_LEFT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_LEFT;
  return path;
}

/*
 * Calculates a dubins path consisting of a right turn, then a straight segment,
 * then a right turn.
 */
Path rsr(double initD, double finalD, double dist) {
  double t = mod(initD - atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);
  double p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(finalD)-sin(initD)));
  double q = mod(mod(-finalD,2*PI) + atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);

  std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_RIGHT;
  return path;
}

/*
 * Calculates a dubins path consisting of a right turn, then a straight segment,
 * then a left turn.
 */
Path rsl(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  double q = mod(mod(finalD, 2*PI) - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_LEFT;
  return path;
}

/*
 * Calculates a dubins path consisting of a left turn, then a straight segment,
 * then a right turn.
 */
Path lsr(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) + 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(-initD + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);
  double q = mod(mod(-finalD, 2*PI) + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);

  std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_LEFT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_RIGHT;
  return path;
}

//Calculates a dubins path consisting of a right turn, then a left turn, then a right turn
Path rlr(double initD, double finalD, double dist) {
  double p = acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(sin(initD)-sin(finalD))));
  double t = mod(initD-atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD)))+(p/2),2*PI);
  double q = mod(initD-finalD-t+p,2*PI);

  std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_LEFT; path.dir3 = DUBINS_RIGHT;
  return path;
}

/* Note:: there was a mistake in the paper discribing how to compute the lrl, I
 * did my best to compute the solution, but this section needs to be tested further
 * Calculates a dubins path consisting of a left turn, then a right turn, then
 * a left turn.
 */
Path lrl(double initD, double finalD, double dist) {
  double p = mod(acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(initD-finalD))),2*PI);
  double t = mod(-initD-atan((cos(initD)-cos(finalD))/(dist+sin(initD)-sin(finalD)))+(p/2),2*PI);
  double q = mod(mod(finalD,2*PI)-initD+2*p,2*PI);

  std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_LEFT; path.dir2 = DUBINS_RIGHT; path.dir3 = DUBINS_LEFT;
  return path;
}



/*
 * Converts a Path into a set of Controls, consisting of a turn direction and
 * distance.
 */
std::vector<Control> dubins::pathToControls(Path dubinsPath) {

  Control control1, control2, control3;
  control1.direction = dubinsPath.dir1;
  control1.distance = dubinsPath.seg1;

  control2.direction = dubinsPath.dir2;
  control2.distance = dubinsPath.seg2;

  control3.direction = dubinsPath.dir3;
  control3.distance = dubinsPath.seg3;
  // std::cout << "First, turn " <<  direction1 << " for: " << control1.distance << " timesteps\n";
  // std::cout << "Then, turn " <<  direction2 << " for: " << control2.distance <<" timesteps\n";
  // std::cout << "Finally, turn " << direction3 <<" for: " << control3.distance <<" timesteps\n";
  return std::vector<Control> { control1, control2, control3 };
}

/*
 * Main function to calculate a dubins path
 * Calls functions to calculate each path individually, finds minimum length
 * path assuming unit turning radius,  then scales path to proper length
 */
Path dubins::calculateDubins(std::vector<Waypoint> waypoints, Waypoint carpoint) {
  Waypoint testpoint = waypoints.front();

  double distance = waypoint_distance(testpoint, carpoint)/scalingFactor_;
  double dubinsAngle = atan((testpoint.y - carpoint.y) / (testpoint.x - carpoint.x));
  printf("\nOur dubins angle is %f\n", dubinsAngle);

  // account for negatives
  if (testpoint.x < 0) { dubinsAngle += PI; }

  printf("\nOur transformed dubins angle is %f\n", dubinsAngle);

  double initDirection = carpoint.direction-dubinsAngle;
  double finalDirection = testpoint.direction-dubinsAngle;

  // Scale our distance, so we calculate dubins path length assuming a unit
  // minimum turning radius
  //  distance = distance / scalingFactor_;

  // Calculate each type of dubins path individually
  Path paths[4] = {
    lsl(initDirection, finalDirection, distance),
    lsr(initDirection, finalDirection, distance),
    rsr(initDirection, finalDirection, distance),
    rsl(initDirection, finalDirection, distance)
  };

  // Finds path of smallest length
  Path dubinsPath = paths[0];
  for (Path path : paths) {
    if (path.length < dubinsPath.length) {
      dubinsPath = path;
    }
  }

  // Rescales path assuming unit turning radius to proper length
  dubinsPath *= scalingFactor_;
  dubinsPath.origin.x = carpoint.x;
  dubinsPath.origin.y = carpoint.y;
  dubinsPath.origin.z = carpoint.direction;
  dubinsPath.rotationAngle = dubinsAngle;

  printf("The minimum path is of length: %f\n", dubinsPath.length);
  printf("Seg 1 is length: %f, .seg 2 is length: %f, .seg 3 is length: %f\n",
    dubinsPath.seg1, dubinsPath.seg2, dubinsPath.seg3);

  return dubinsPath;
}

/*
 * Left turn operator, given an initial waypoint, position, and distance to
 * travel. Returns a point corresponding to a maxim left turn.
 */
cv::Point3d dubins::leftTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;
  newPos.x = x + sin(theta + dist) - sin(theta);
  newPos.y = y - cos(theta + dist) + cos(theta);
  newPos.z = theta + dist;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";
  return newPos;
}

/*
 * Right turn operator, given an initial waypoint, position, and distance to
 * travel. Returns a point corresponding to a maxim right turn.
 */
cv::Point3d dubins::rightTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;

  newPos.x = x - sin(theta - dist) + sin(theta);
  newPos.y = y + cos(theta - dist) - cos(theta);
  newPos.z = theta - dist;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";
  return newPos;
}


/*
 * Straight turn operator, given an initial waypoint, position, and distance to
 * travel. Returns a point corresponding to a straight 'turn'.
 */
cv::Point3d dubins::straightTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;
  newPos.x = x + dist * cos(theta);
  newPos.y = y + dist * sin(theta);
  newPos.z = theta;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";
  return newPos;
}
