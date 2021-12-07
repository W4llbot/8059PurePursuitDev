#ifndef _8059_PP_PATH_HPP_
#define _8059_PP_PATH_HPP_
#include "Node.hpp"
#include <vector>
#define inPerDeg 0.024182854666401 //use baseOdom to tune?
// #define powerToDegsPerMs 0.0104 //empirical use excel
// #define powerToInPerMs (powerToDegsPerMs * inPerDeg)

#define RPMToInPerMs 1/60/1000*360*inPerDeg
#define MAXRPMV 100.0
#define MAXRPMA .1

#define voltageToPower 127/12000
// #define MAXPOWV 20.0 //power
// #define MAXPOWA 1 //.5 power every ms

// #define MAXV MAXPOWV*powerToInPerMs
// #define MAXA MAXPOWA*powerToInPerMs
#define globalMaxV MAXRPMV * RPMToInPerMs
#define globalMaxA MAXRPMA * RPMToInPerMs

//INJECT
#define SPACING 1
//SMOOTH
#define TOLERANCE 0.001
//MAXV
#define K 0.005
class Path{
private:
  std::vector<Node> wps;
  std::vector<Node> injWps;

  std::vector<Node> smoWps;
  double w_data, w_smooth, lookAhead;
  int n;

  std::vector<double> dist;
  std::vector<double> curv;
  std::vector<double> maxV;
  std::vector<double> targV;

public:
  Path();
  Path(std::vector<Node> p_wps);
  Node getSmoWp(int i);
  double getMaxV(int i);
  double getTargV(int i);
  int getN();
  double getLookAhead();
  void inject();
  void smooth();
  void calcDist();
  void calcCurvature();
  void calcMaxV();
  void calcTargV();
  void setWps(std::vector<Node> p_wps, double p_w_data, double p_w_smooth, double p_lookAhead);
  void debugPoint(int i);
};
#endif
