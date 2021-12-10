#include "main.h"
#define kV 3978
#define kA 50000
#define kP 1000

bool enablePP = false;
bool reverse = false;
Path path;

int closestPointIndex = 0;
double lastFracIndex = 0;

void drive(double l, double r){
  Motor FL (FLPort);
  Motor BL (BLPort);
  Motor FR (FRPort);
  Motor BR (BRPort);
  FL.move(l);
  BL.move(l);
  FR.move(r);
  BR.move(r);
}

void resetPP() {
  closestPointIndex = 0;
  lastFracIndex = 0;
}

void basePP(std::vector<Node> wps, double p_w_data, double p_w_smooth, double p_lookAhead, bool p_reverse){
  path.setWps(wps, p_w_data, p_w_smooth, p_lookAhead);
  reverse = p_reverse;
}

void waitPP(double cutoff){
  int start = millis();
  Node target = path.getSmoWp(path.getN()-1);
  while(distance(position, target)>=LEEWAY || fabs(measuredV) > 0.0001) delay(5);

  resetPP();
  enablePP = false;

  printf("I stopped :)\n");
}

void PPControl(void * ignore){
  Node lookAheadNode;

  // unit: in/ms
  double targV = 0, targVL = 0, targVR = 0;
  double prevTargVL = 0, prevTargVR = 0;
  // unit: in/ms^2
  double targAL = 0, targAR = 0;

  int count = 0;

  while(true){
    if(count % 10 == 0) printf("status: %s\n", (enablePP? "enabled": "disabled"));

    if(enablePP) {
      // FIND CLOSEST POINT
      double minDist = INF;
      for(int i = closestPointIndex; i < path.getN(); ++i){
        double d = distance(position, path.getSmoWp(i));
        if(d < minDist){
          minDist = d;
          closestPointIndex = i;
        }
      }
      if(count % 10 == 0) {
        printf("closest point: \n");
        // path.getSmoWp(closestPointIndex).print();
        path.debugPoint(closestPointIndex);
      }

      // ===================================================================================

      // FIND LOOK AHEAD POINT
      for(int i = floor(lastFracIndex);i<=path.getN()-2;++i){ // starting from int i = 0 is technically less efficient
        // iterate every path
        Node start = path.getSmoWp(i);
        Node end = path.getSmoWp(i+1);
        std::vector<double> l = position.findLookAhead(start, end, path.getLookAhead());
        if(l[0]){
          // if there's an intersection
          // calculate fractional index
          double fracIndex = i + l[1];
          if(fracIndex >= lastFracIndex){
            lookAheadNode = start + (end - start) * l[1];
            lastFracIndex = fracIndex;
            break;
          }
        } // else keep the same lookaheadnode
      }

      if(count % 10 == 0) {
        printf("Curr point:");
        position.print();
        printf("look ahead point:");
        lookAheadNode.print();
      }

      // ===================================================================================

      // calculate moveCurvature
      // under "Curvature of Arc" header
      // calculate angle from x-axis, counter clockwise as positive
      double practicalAngle = reverse ? angle+PI:angle;

      double a = -tan(practicalAngle);
      double b = 1;
      double c = tan(practicalAngle)*position.getX() - position.getY();
      double xabs = fabs(a * lookAheadNode.getX() + b * lookAheadNode.getY() + c)/sqrt(a*a + b*b);
      double crossProduct = sin(practicalAngle)*(lookAheadNode.getX() - position.getX()) - cos(practicalAngle)*(lookAheadNode.getY() - position.getY());
      double sign = crossProduct >= 0 ? 1 : -1;
      double moveCurvature = sign*2*xabs/(path.getLookAhead()*path.getLookAhead());

      // ===================================================================================

      // find target velocities
      /**
      * point (2): To get the target velocity take the target velocity associated with the closest point, and
      * constantly feed this value through the rate limiter to get the acceleration-limited target velocity.
      */
      double targVClosest = reverse ? -path.getTargV(closestPointIndex) : path.getTargV(closestPointIndex);
      // rate limiter
      // targV = targV + abscap(targVClosest, globalMaxA); //might use v + abscap instead of targV + abscap?
      targV = targVClosest;
      if(count % 10 == 0) printf("TargV: %.5f, MAXV: %.5f\n", targV, globalMaxV);
      targVL = targV*(2 + moveCurvature*baseWidth)/2;
      targVR = targV*(2 - moveCurvature*baseWidth)/2;
      if(count % 10 == 0) printf("Move Curvature: %.5f\n", moveCurvature);
      if(count % 10 == 0) printf("TargVL: %.5f\tTargVR: %.5f\n", targVL, targVR);

      // ===================================================================================

      // motor controller
      targAL = (targVL - prevTargVL)/dT;
      targAR = (targVR - prevTargVR)/dT;
      // feedforward terms
      double ffL = kV * targVL + kA * targAL;
      double ffR = kV * targVR + kA * targAR;
      // feedback terms
      double fbL;
      double fbR;

      if(reverse) {
        fbL = kP * (targVL - measuredVR);
        fbR = kP * (targVR - measuredVL);
      }else {
        fbL = kP * (targVL - measuredVL);
        fbR = kP * (targVR - measuredVR);
      }

      // set power
      if(reverse) {
        if(count % 10 == 0) printf("PowerL: %4.2f\tPowerR: %4.2f\n", (ffR + fbR), (ffL + fbL));
        drive((ffR + fbR), (ffL + fbL));
      }else {
        if(count % 10 == 0) printf("PowerL: %4.2f\tPowerR: %4.2f\n", (ffL + fbL), (ffR + fbR));
        drive((ffL + fbL), (ffR + fbR));
      }
      // handling prev
      prevTargVL = targVL;
      prevTargVR = targVR;
      // debugging

      if(count % 10 == 0) printf("TargV: %4.5f\tMeasuredv: %4.5f\n\n", targV, measuredV);
      count++;
      delay(dT);
    }
  }
}
