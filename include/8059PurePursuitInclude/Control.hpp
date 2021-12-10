#ifndef _8059_PP_CONTROL_HPP_
#define _8059_PP_CONTROL_HPP_
#define LEEWAY 1
#define DEFAULT_W_SMOOTH 0.75
#define DEFAULT_W_DATA 1-DEFAULT_W_SMOOTH
#define DEFAULT_LOOK_AHEAD 24
#define baseWidth 16.18003749869522
#define dT 5
extern bool enablePP;

void drive(double l, double r);
void basePP(std::vector<Node> wps, double p_w_data = DEFAULT_W_DATA, double p_w_smooth = DEFAULT_W_SMOOTH, double p_lookAhead = DEFAULT_LOOK_AHEAD, bool p_reverse = false);
void waitPP(double cutoff);
void PPControl(void * ignore);
#endif
