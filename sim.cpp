// =============================================================================
//  Car drivetrain simulation model -- implementation.
// =============================================================================
#include "sim.h"

#include <cmath>
#include <algorithm>

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

// ----------------------------- engine ---------------------------------------
void Engine::sortCurve(){
    std::sort(curve.begin(),curve.end(),
              [](const CurvePoint&a,const CurvePoint&b){ return a.rpm<b.rpm; });
}

double Engine::torqueAt(double rpm) const {
    if(curve.empty()) return 0.0;
    if(rpm<=curve.front().rpm) return curve.front().nm;
    if(rpm>=curve.back().rpm)  return curve.back().nm;
    for(size_t i=1;i<curve.size();++i){
        if(rpm<=curve[i].rpm){
            double dr=curve[i].rpm-curve[i-1].rpm;
            double t = dr>1e-9 ? (rpm-curve[i-1].rpm)/dr : 0.0;
            return curve[i-1].nm + t*(curve[i].nm-curve[i-1].nm);
        }
    }
    return curve.back().nm;
}

double Engine::netTorque(double throttle) const {
    double rpm = omega*RAD2RPM;
    double thr = throttle;
    if(rpm>redline) thr=0.0;                 // hard rev limiter
    double wot   = torqueAt(rpm);
    double drive = wot*thr;
    double fric  = 14.0 + 0.0045*rpm;        // pumping/engine-brake torque
    double idle  = 0.0;                       // simple idle-assist controller
    if(rpm<idleRPM) idle = (idleRPM-rpm)*0.45;
    return drive - fric + idle;
}

// ----------------------------- clutch ---------------------------------------
double Clutch::torque(double dOmega) const {
    return capacity()*engagement*std::tanh(dOmega/band);
}

// ----------------------------- tire (Pacejka) -------------------------------
Surface SURFACES[3] = { {"DRY",1.00}, {"WET",0.60}, {"ICE",0.15} };

double Tire::D(double mu,double Fz) const { return mu*Fz; }
double Tire::B(double mu,double Fz) const {
    double d=std::max(D(mu,Fz),1e-3);
    return Kstiff/(C*d);
}
double Tire::kappaPeak(double mu,double Fz) const {
    return std::tan(PI_/(2.0*C))/std::max(B(mu,Fz),1e-6);
}
double Tire::force(double kappa,double mu,double Fz) const {
    double b=B(mu,Fz), d=D(mu,Fz);
    double phi = b*kappa - E*(b*kappa - std::atan(b*kappa));
    return d*std::sin(C*std::atan(phi));
}

// ----------------------------- vehicle --------------------------------------
Vehicle::Vehicle(){
    // default RWD car: front pair (free) + rear pair (driven), one open diff
    addWheel( 1.30,  0.80, 0.31, false);   // 0 FL
    addWheel( 1.30, -0.80, 0.31, false);   // 1 FR
    addWheel(-1.30,  0.80, 0.31, true );   // 2 RL
    addWheel(-1.30, -0.80, 0.31, true );   // 3 RR
    Differential d; d.a=2; d.b=3; d.mode=DIFF_OPEN;
    diffs.push_back(d);
    eng.sortCurve();
}

void Vehicle::reset(){
    // reset dynamic state only -- keep the drivetrain configuration
    v=x=accel=0.0;
    eng.omega   = eng.idleRPM*RPM2RAD;
    eng.stalled = false;
    box.gear    = 1;
    for(Wheel& wh : wheels){ wh.omega=wh.kappa=wh.angle=0.0; wh.Fx=0.0; }
}

int Vehicle::drivenCount() const {
    int c=0; for(const Wheel& wh: wheels) if(wh.driven) c++; return c;
}

void Vehicle::addWheel(double px,double pz,double r,bool driven){
    Wheel wh; wh.px=px; wh.pz=pz; wh.r=r; wh.driven=driven;
    wheels.push_back(wh);
}

void Vehicle::removeWheel(int idx){
    if(idx<0 || idx>=(int)wheels.size()) return;
    wheels.erase(wheels.begin()+idx);
    // drop any differential that referenced it; shift higher indices down
    for(int i=(int)diffs.size()-1;i>=0;--i){
        if(diffs[i].a==idx || diffs[i].b==idx){ diffs.erase(diffs.begin()+i); continue; }
        if(diffs[i].a>idx) diffs[i].a--;
        if(diffs[i].b>idx) diffs[i].b--;
    }
}

void Vehicle::addDiff(){
    Differential d;
    // default to the first two distinct wheels
    d.a = 0;
    d.b = wheels.size()>1 ? 1 : 0;
    diffs.push_back(d);
}

void Vehicle::removeDiff(int idx){
    if(idx<0 || idx>=(int)diffs.size()) return;
    diffs.erase(diffs.begin()+idx);
}

void Vehicle::step(double dt,double throttle,double brake){
    double mu = SURFACES[surf].mu;
    int N = (int)wheels.size();
    if(N==0){ v=0; return; }

    // --- weight: distribute static load, then transfer front<->rear on accel
    // (skipped in external mode -- the 3D suspension supplies each wheel's Fz)
    double Wt = mass*g;
    if(!external){
        double staticEach= Wt/N;
        double dFz       = mass*accel*cgH/wheelbase;   // transferred front->rear
        int nf=0,nr=0;
        for(const Wheel& wh: wheels){ if(wh.px>0) nf++; else nr++; }
        for(Wheel& wh: wheels){
            double t = 0.0;
            if(wh.px>0 && nf>0) t = -dFz/nf;           // front sheds load
            else if(wh.px<=0 && nr>0) t = +dFz/nr;     // rear gains load
            wh.Fz = clampd(staticEach + t, 0.0, 1e6);
        }
    }

    // --- engine vs driveline coupling through the clutch --------------------
    double n = box.n();
    double omegaCarrier = 0.0;                      // mean driven-wheel speed
    int nd = drivenCount();
    if(nd>0){
        for(const Wheel& wh: wheels) if(wh.driven) omegaCarrier += wh.omega;
        omegaCarrier /= nd;
    }
    double omegaIn = omegaCarrier*n;                // (0 in neutral -> n==0)

    // a stalled (dead) engine produces no combustion torque
    double Te = eng.stalled ? 0.0 : eng.netTorque(throttle);

    double Tc;
    if(box.gear==0 || nd==0){ Tc=0.0; clu.lockedish=false; }  // no load path
    else{
        double dW = eng.omega - omegaIn;
        Tc = clu.torque(dW);
        clu.lockedish = !eng.stalled && std::fabs(dW) < 1.5 && clu.engagement>0.6;
    }

    // engine EoM:  I_e * dω = T_engine - T_clutch
    // No stall floor: if the load (a too-tall gear, clutch dumped at low revs)
    // drags the engine below its stall speed, it dies -- torque cuts out and
    // the car fails to launch, exactly like the real thing.
    eng.omega += dt*(Te - Tc)/eng.I;
    eng.omega  = std::max(eng.omega, 0.0);
    if(!eng.stalled && eng.omega < eng.stallRPM*RPM2RAD) eng.stalled = true;
    // restart (idle) once the load is removed: neutral or clutch pedal in
    if(eng.stalled && (box.gear==0 || clu.engagement < 0.15)){
        eng.stalled = false;
        eng.omega   = eng.idleRPM*RPM2RAD;
    }

    // total axle torque, split equally across the driven wheels (open behaviour)
    double Twheels = Tc*n*box.eff;
    double Tdriven = nd>0 ? Twheels/nd : 0.0;

    // --- differential coupling: soft-lock torque transferred fast -> slow ----
    std::vector<double> Tlock(N,0.0);
    for(const Differential& d : diffs){
        if(d.a<0||d.b<0||d.a>=N||d.b>=N||d.a==d.b) continue;
        double cap = d.effectiveCap();
        if(cap<=0.0) continue;
        double dWw = wheels[d.a].omega - wheels[d.b].omega;
        double T   = cap*std::tanh(dWw/d.band);
        Tlock[d.a] -= T;                            // brake the faster wheel
        Tlock[d.b] += T;                            // drive the slower wheel
    }

    // --- per-wheel: tire slip (relaxation) -> force -> reaction torque -------
    double Fx_total=0.0;
    for(int i=0;i<N;i++){
        Wheel& wh=wheels[i];

        // single-point relaxation model:  sigma*dκ/dt = Vsx - |v|*κ
        double Vsx = wh.omega*wh.r - v;
        double dk  = (Vsx - std::fabs(v)*wh.kappa)/sigma;
        wh.kappa  += dt*dk;
        wh.kappa   = clampd(wh.kappa,-4.0,4.0);

        wh.Fx = tire.force(wh.kappa, mu, wh.Fz);
        double Treact = wh.Fx*wh.r;

        double Tdrive = wh.driven ? Tdriven : 0.0;

        // brake torque opposes spin; don't drive omega through zero in one step
        double Tbrake = brake*1600.0;
        double Tb = -sgn(wh.omega)*Tbrake;
        if(std::fabs(wh.omega) < 1e-3) Tb = 0.0;

        wh.omega += dt*(Tdrive + Tlock[i] - Treact + Tb)/wh.I;
        wh.angle += wh.omega*dt;

        Fx_total += wh.Fx;
    }

    // --- chassis longitudinal dynamics --------------------------------------
    // In external mode the 3D rigid body integrates motion (and feeds v back in
    // before the next call), so we only run this simple 1D model standalone.
    if(!external){
        double drag = 0.5*rho*Cd*A*v*std::fabs(v);
        double roll = Crr*Wt*sgn(v);
        double net  = Fx_total - drag - roll;
        accel = net/mass;
        v    += dt*accel;
        if(std::fabs(v)<1e-3 && std::fabs(net)<roll+1.0) v=0.0; // settle at rest
        x    += dt*v;
    }
}
