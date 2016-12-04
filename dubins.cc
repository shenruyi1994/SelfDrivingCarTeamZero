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


double mod(double a, double b) {
  double ret = fmod(a,b);
  if (ret < 0)
    ret += b;
  return ret;
}

Path  lsl(double initD, double finalD, double dist){
  // length of first left turn, p is # of timesteps for straight, q is # of timesteps for second left turn;                                                                                              
  double t, p, q;
  Path lslPath;

  t = mod((-initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));
  q = mod(finalD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  lslPath.seg1 = t;
  lslPath.seg2 = p;
  lslPath.seg3 = q;
  lslPath.length = t+p+q;
  lslPath.type = "lsl";
  std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

  return lslPath;

}

Path rsr(double initD, double finalD, double dist) {
  double t,p,q;
  Path rsrPath;
  t = mod(initD - atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(finalD)-sin(initD)));
  q = mod(finalD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  rsrPath.seg1 = t;
  rsrPath.seg2 = p;
  rsrPath.seg3 = q;
  rsrPath.length = t+p+q;
  rsrPath.type = "rsr";

  std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  return rsrPath;
}


Path rsl(double initD, double finalD, double dist) {
  double t,p,q;
  Path rslPath;

  p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  q = mod(mod(finalD, 2*PI) - atan((cos(initD)-cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  rslPath.seg1 = t;
  rslPath.seg2 = p;
  rslPath.seg3 = q;
  rslPath.length = t+p+q;
  rslPath.type = "rsl";
  std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  return rslPath;
   

}

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
  lsrPath.type = "lsr";

  std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  return lsrPath;
  
}

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
  rlrPath.type = "rlr";

  std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  return rlrPath;
}

//Note there was a typo in the paper discribing how to compute the lrl,
//I did my best to compute the solution, but this section needs to be tested further
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
  lrlPath.type = "lrl";
  std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  return lrlPath;
}



int dubins::calculateDubins(Waypoints* waypoints) {
  //direction in radians
  double initDirection = 0;
  cv::Point initPosition = cv::Point(0,0);

  //direction in radians
  double finalDirection = 0;
  cv::Point initPoint = cv::Point(0,10);
  double distance = 100;

  //double distance = sdcLLC::car_->sdcCar::GetDistance(finalPosition);

  Path lslP = lsl(initDirection, finalDirection, distance);
  Path lsrP = lsr(initDirection, finalDirection, distance);
  Path rsrP = rsr(initDirection, finalDirection, distance); 
  Path rslP = rsl(initDirection, finalDirection, distance);
  Path rlrP = rlr(initDirection, finalDirection, distance);
  Path lrlP = lrl(initDirection, finalDirection, distance);

  return 0;
}
