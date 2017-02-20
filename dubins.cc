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
  scalingFactor_ = 2 * MIN_TURNING_RADIUS * PI;
}

/* Positive modulo operator
 * Intput: two doubles a and b
 * Output: (a mod b) with posive buffer
 */ 
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

  // std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

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

  // std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_RIGHT;
  return path;
}

/* --Calculates an rsl dubins path--
 * Input: Initial direction of motion (radians), final direction of motion (radians), 
 *   distance between initial point and final point (double)
 * Output: right segment, straight segment, left segment, dubins path
 */
Path rsl(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  double q = mod(mod(finalD, 2*PI) - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  // std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_LEFT;
  return path;
}

/* --Calculates an lsr dubins path--                                                                                         
 * Input: Initial direction of motion (radians), final direction of motion (radians),
 *   distance between initial point and final point (double)
 * Output: left segment, straight segment, right segment dubins path
 */
Path lsr(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) + 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(-initD + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);
  double q = mod(mod(-finalD, 2*PI) + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);

  // std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_LEFT; path.dir2 = DUBINS_STRAIGHT; path.dir3 = DUBINS_RIGHT;
  return path;
}

/* --Calculates an rsl dubins path--                                                                                          
 * Input: Initial direction of motion (radians), final direction of motion (radians),                                          
 *   distance between initial point and final point (double)                                                                   
 * Output: right segment straight segment left sgment dubins path                                                              
 */
Path rlr(double initD, double finalD, double dist) {
  double p = acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(sin(initD)-sin(finalD))));
  double t = mod(initD-atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD)))+(p/2),2*PI);
  double q = mod(initD-finalD-t+p,2*PI);

  // std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_RIGHT; path.dir2 = DUBINS_LEFT; path.dir3 = DUBINS_RIGHT;
  return path;
}

/* --Calculates an lsl dubins path--                                                                                      
 * Input: Initial direction of motion (radians), final direction of motion (radians),                                          
 *   distance between initial point and final point (double)                                                                   
 * Output: left segment, straight segment, left segment dubins path 
 */
Path lrl(double initD, double finalD, double dist) {
  double p = mod(acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(initD-finalD))),2*PI);
  double t = mod(-initD-atan((cos(initD)-cos(finalD))/(dist+sin(initD)-sin(finalD)))+(p/2),2*PI);
  double q = mod(mod(finalD,2*PI)-initD+2*p,2*PI);

  // std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  struct Path path {};
  path.seg1 = t; path.seg2 = p; path.seg3 = q;
  path.length = t + p + q;
  path.dir1 = DUBINS_LEFT; path.dir2 = DUBINS_RIGHT; path.dir3 = DUBINS_LEFT;
  return path;
}

/*
 * --Converts a Path into a set of Controls--
 * Input: dubinsPath consisting of path type and segment lengths
 * Oput: Vector of controls, where a control has a direction (-1, 0, 1) and a distanace 
 */
std::vector<Control> dubins::pathToControls(Path dubinsPath) {
  Control control1, control2, control3;
  control1.direction = dubinsPath.dir1;
  control1.distance = dubinsPath.seg1;

  control2.direction = dubinsPath.dir2;
  control2.distance = dubinsPath.seg2;

  control3.direction = dubinsPath.dir3;
  control3.distance = dubinsPath.seg3;

  /* 
   std::cout << "First, turn " <<  direction1 << " for: " << control1.distance << " timesteps\n";
   std::cout << "Then, turn " <<  direction2 << " for: " << control2.distance <<" timesteps\n";
   std::cout << "Finally, turn " << direction3 <<" for: " << control3.distance <<" timesteps\n";
  */
  return std::vector<Control> { control1, control2, control3 };
}

/*
 * Main function to calculate a dubins path
 * Calls functions to calculate each path individually, finds minimum length
 * path assuming unit turning radius,then scales path to proper length
 */
Path dubins::calculateDubins(Waypoint waypoint, Waypoint carPoint, double minRadius) {
  scalingFactor_ = minRadius;

  // carPoint.x = 85;
  // carPoint.y = 15;
  //carPoint.direction = PI;

  // waypoint.x = 95;
  // waypoint.y = 10;
  // waypoint.direction = PI;
  double distance = coord_distance(waypoint, carPoint)/scalingFactor_;

  double dubinsAngle = atan((waypoint.y - carPoint.y) / (waypoint.x - carPoint.x));

  //rotates angle into quadrant 2
  if(waypoint.x-carPoint.x < 0 && waypoint.y-carPoint.y >= 0){dubinsAngle = PI+dubinsAngle;}
  //rotates angle to quadrant 3
  if(waypoint.x-carPoint.x < 0 && waypoint.y-carPoint.y < 0){dubinsAngle += PI;}
  //rotates angle to quadrant 4
  if(waypoint.x-carPoint.x >=0 && waypoint.y-carPoint.y < 0){dubinsAngle = 2*PI+dubinsAngle;}

  // printf("\nOur transformed dubins angle is %f\n", dubinsAngle);

  double initDirection = carPoint.direction-dubinsAngle;
  double finalDirection = waypoint.direction-dubinsAngle;

  // Scale our distance, so we calculate dubins path length assuming a unit
  // minimum turning radius
  // distance = distance / scalingFactor_;

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

  //Rescales path assuming unit turning radius to proper length
   dubinsPath *= scalingFactor_;
   dubinsPath.origin.x = carPoint.x;
   dubinsPath.origin.y = carPoint.y;
   dubinsPath.origin.z = carPoint.direction;
   dubinsPath.rotationAngle = dubinsAngle;

   //printf("The minimum path is of length: %f\n", dubinsPath.length);
   // printf("Seg 1 is length: %f, .seg 2 is length: %f, .seg 3 is length: %f\n",
   // dubinsPath.seg1, dubinsPath.seg2, dubinsPath.seg3);
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
newPos.z = mod(theta + dist, 2*PI);

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
  newPos.z = mod(theta - dist,2*PI);


  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";
  return newPos;
}

/* Straight turn operator
 * Input: Initial wayaypoint (x,y,theta), and dist, where theta
 *  is initial direction of motion and dist is the distance to apply the turn for.
 *  travel along a maximum turn. Returns a point corresponding to a straight 'turn'.
 * Output: new position (x,y,direction of motion) after performing the turn
 */
cv::Point3d dubins::straightTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;

  newPos.x = x + dist * cos(theta);
  newPos.y = y + dist * sin(theta);
  newPos.z = mod(theta,2*PI);

  /* std::cout << "Our new x coord is: " << newPos.x << "/n";
  std::cout << "Our new y coord is: " << newPos.y << "/n"; 
  */

  return newPos;
}
