#include "sdcLLC.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"
#include "sdcAngle.hh"
#include "dubins.hh"
#include "globals.hh"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace gazebo;


dubins::dubins(){
}

//True mod function that does not return negative values
double mod(double a, double b) {
  double ret = fmod(a,b);
  if (ret < 0)
    ret += b;
  return ret;
}

//Calculates a dubins path consisting of left turn, then straight segment, then left turn
Path  lsl(double initD, double finalD, double dist){
  //p is length of first turn, q is length of second turn, q is length of third turn
  double t, p, q;
  Path lslPath;

  t = mod((-initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));
  q = mod(finalD - atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  lslPath.seg1 = t;
  lslPath.seg2 = p;
  lslPath.seg3 = q;
  lslPath.length = t+p+q;
  lslPath.type = lslT;

  std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

  return lslPath;
}


//Calculates a dubins path consisting of a right turn, then a straight segment, then a right turn
Path rsr(double initD, double finalD, double dist) {
  double t,p,q;
  Path rsrPath;

  t = mod(initD - atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(finalD)-sin(initD)));
  q = mod(mod(-finalD,2*PI) + atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);

  rsrPath.seg1 = t;
  rsrPath.seg2 = p;
  rsrPath.seg3 = q;
  rsrPath.length = t+p+q;
  rsrPath.type = rsrT;

  std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  return rsrPath;
}

//Calculates a dubins path consisting of a right turn, then a straight segment, then a left turn
Path rsl(double initD, double finalD, double dist) {
  double t,p,q;
  Path rslPath;

  p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  q = mod(mod(finalD, 2*PI) - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  rslPath.seg1 = t;
  rslPath.seg2 = p;
  rslPath.seg3 = q;
  rslPath.length = t+p+q;
  rslPath.type = rslT;

  std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  return rslPath;
}

//Calculates a dubins path consisting of a left turn, then a straight segment, then a right turn
Path lsr(double initD, double finalD, double dist) {
  double t,p,q;
  Path lsrPath;

  p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) + 2*dist*(sin(initD) + sin(finalD)));
  t = mod(-initD + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);
  q = mod(mod(-finalD, 2*PI) + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);

  lsrPath.seg1 = t;
  lsrPath.seg2 = p;
  lsrPath.seg3 = q;
  lsrPath.length = t+p+q;
  lsrPath.type = lsrT;

  std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  return lsrPath;
}

//Calculates a dubins path consisting of a right turn, then a left turn, then a right turn
Path rlr(double initD, double finalD, double dist) {
  double t,p,q;
  Path rlrPath;

  p = acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(sin(initD)-sin(finalD))));
  t = mod(initD-atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD)))+(p/2),2*PI);
  q = mod(initD-finalD-t+p,2*PI);
  
  rlrPath.seg1 = t;
  rlrPath.seg2 = p;
  rlrPath.seg3 = q;
  rlrPath.length = t+p+q;
  rlrPath.type = rlrT;

  std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  return rlrPath;
}

//Note:: there was a mistake in the paper discribing how to compute the lrl,
//I did my best to compute the solution, but this section needs to be tested further
//Calculates a dubins path consisting of a left turn, then a right turn, then a left turn
Path lrl(double initD, double finalD, double dist) {
  double t,p,q;
  Path lrlPath;
  p = mod(acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(initD-finalD))),2*PI);
  t = mod(-initD-atan((cos(initD)-cos(finalD))/(dist+sin(initD)-sin(finalD)))+(p/2),2*PI);
  q = mod(mod(finalD,2*PI)-initD+2*p,2*PI);

  lrlPath.seg1 = t;
  lrlPath.seg2 = p;
  lrlPath.seg3 = q;
  lrlPath.length = t+p+q;
  lrlPath.type = lrlT;

  std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  return lrlPath;
}

//Custom compare functor to compare lengths of dubins path
bool comparePath(Path &a, Path &b) {
  return a.length< b.length;
}

//Returns the minimum lenght dubins path
//Right now we are only checking the paths of form CSC, will implement CCC paths if needed
Path minPath(Path a, Path b, Path c, Path d){
  std::vector<Path> paths;

  paths.push_back(a);
  paths.push_back(b);
  paths.push_back(c);
  paths.push_back(d);
  
  sort(paths.begin(), paths.end(), comparePath);

  return paths.front();
}



//Scales a path length by minimum turning radius, since we initially divided the distance to our waypoint by our minimum turning radius so we could calulate a dubins path with turning radius of 1 to make our calulations more simple
Path scalePath(Path dubinsPath){
  dubinsPath.seg1 = dubinsPath.seg1*MIN_TURNING_RADIUS;
  dubinsPath.seg2 = dubinsPath.seg2*MIN_TURNING_RADIUS;
  dubinsPath.seg3 = dubinsPath.seg3*MIN_TURNING_RADIUS;
  dubinsPath.length = dubinsPath.length*MIN_TURNING_RADIUS;

  return dubinsPath;
}

Controls pathToControls(Path dubinsPath){
  double updateRate = 1000;
  double velocity = 6;
  
  double direction1, direction2, direction3;
  type type = dubinsPath.type;
  


  switch(type){
  case lslT:
    direction1 = -1;
    direction2 = 0;
    direction3 = -1;
    break;
  case lsrT:
    direction1 = -1;
    direction2 = 0;
    direction3 = 1;
    break;
  case rslT:
    direction1 = 1;
    direction2 = 0;
    direction3 = -1;
    break;
  case rsrT:
     direction1 = 1;
    direction2 = 0;
    direction3 = 1;
    break;
  case lrlT: 
    direction1 = -1;
    direction2 = 1;
    direction3 = -1;
    break;
  case rlrT:
    direction1 = 1;
    direction2 = -1;
    direction3 = 1;
    break;
  }
    

  Control control1, control2, control3;
  Controls controls;

  control1.control.first = direction1;
  control1.control.second= dubinsPath.seg1*updateRate/velocity;
  controls.controls.push_back(control1);

  control2.control.first = direction2;
  control2.control.second= dubinsPath.seg2*updateRate/velocity;
  controls.controls.push_back(control2);

  control3.control.first = direction3;
  control3.control.second= dubinsPath.seg3*updateRate/velocity;
  controls.controls.push_back(control3);
  std::cout << "First, turn " <<  direction1 << " for: " << control1.control.second << " timesteps\n";
  std::cout << "Then, turn " <<  direction2 << " for: " << control2.control.second <<" timesteps\n";
  std::cout << "Finally, turn " << direction3 <<" for: " << control3.control.second <<" timesteps\n";
  return controls;
}

//Main function to calculate a dubins path
//Calls functions to calculate each path individually, finds minimum lenght path assuming unit turning radius,  then scales path to proper length
Path dubins::calculateDubins(std::vector<Waypoint> waypoints) {
  
  std::vector<Waypoint> testpoints;
  Waypoint testpoint;
  testpoint.x = 5;
  testpoints.push_back(testpoint);
  
  //testpoints.addWaypoint(waypoints.waypoints_.front(), testpoints.waypoints_);
  
  std::cout << testpoints.front().x;

  // testpoints.waypoints_ = testpoints.waypoints_.Waypoint(waypoints);
  //direction in radians
  double initDirection = 0;
  //direction in radians
  double finalDirection =1;

  //distance to object
  //TODO: Replace this with a dunction that calculates distance between car and Waypoint
  double distance = 10;

  //Scale our distance, so we calculate dubins path length assuming a unit minimum turning radius 
  distance = distance/MIN_TURNING_RADIUS;

  //Calculate each type of dubins path individually
  Path lslP = lsl(initDirection, finalDirection, distance);
  Path lsrP = lsr(initDirection, finalDirection, distance);
  Path rsrP = rsr(initDirection, finalDirection, distance); 
  Path rslP = rsl(initDirection, finalDirection, distance);
  // Path rlrP = rlr(initDirection, finalDirection, distance);
  // Path lrlP = lrl(initDirection, finalDirection, distance);

  //Returns path of smallest length
  Path dubinsPath = minPath(lslP,lsrP,rsrP,rslP);

  //Rescales path assuming unit turning radius to proper length
  dubinsPath = scalePath(dubinsPath);


  //Controls controls = pathToControls(dubinsPath);

  std::cout << "The minimum path of type: " << dubinsPath.type << " is of length: " << dubinsPath.length << "\n";
  std::cout << "Seg 1 is length: " << dubinsPath.seg1 << "  Seg 2 is length: " << dubinsPath.seg2 << "  Seg 3 is length: " << dubinsPath.seg3 << "\n";
  
  // return controls;
  return dubinsPath;
}

