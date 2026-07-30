// =============================================================================
//  sim.cpp -- the shared pieces of the vehicle simulation API (see sim.h).
//
//  Only the maths helpers every front-end uses and the named-surface table live
//  here.  Each concrete vehicle model brings its own translation unit; the Ford
//  Transit manual drivetrain is in drivetrain.cpp.
// =============================================================================
#include "sim.h"

// ----------------------------- small helpers --------------------------------
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
double sgn(double v){ return (v>0)-(v<0); }

double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x){
    if(x<=xs.front()) return ys.front();
    if(x>=xs.back())  return ys.back();
    for(size_t i=1;i<xs.size();++i){
        if(x<=xs[i]){
            double t=(x-xs[i-1])/(xs[i]-xs[i-1]);
            return ys[i-1]+t*(ys[i]-ys[i-1]);
        }
    }
    return ys.back();
}

static const double PI_ = 3.14159265358979323846;
const double RAD2RPM = 60.0/(2.0*PI_);
const double RPM2RAD = (2.0*PI_)/60.0;

// ----------------------------- surfaces -------------------------------------
// Hosts classify their ground with this table and pass the chosen mu down per
// contact (vsim::ContactIn::mu); no simulation reads it directly.
Surface SURFACES[3] = { {"DRY",1.00}, {"WET",0.60}, {"ICE",0.15} };


