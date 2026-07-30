// =============================================================================
//  drivetrain.cpp -- Ford Transit Mk7 330M manual drivetrain (see drivetrain.h).
//
//  Implements vsim::IVehicleSim.  step() is the API entry point: it applies the
//  driver's Command, refreshes each corner from its ground ContactIn, then
//  subdivides dt and runs integrate() -- the actual physics -- as many times as
//  the stiff clutch/driveline coupling needs.
// =============================================================================
#include "drivetrain.h"

#include <cmath>
#include <algorithm>

namespace vsim {

static const double PI_ = 3.14159265358979323846;

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
    // diesel: high compression -> strong pumping / engine-brake torque
    double fric  = 20.0 + 0.006*rpm;         // pumping/engine-brake torque
    double idle  = 0.0;                       // simple idle-assist controller
    if(rpm<idleRPM) idle = (idleRPM-rpm)*0.45;
    return drive - fric + idle;
}

// ----------------------------- clutch ---------------------------------------
double Clutch::torque(double dOmega) const {
    return capacity()*engagement*std::tanh(dOmega/band);
}

// ----------------------------- tire (Pacejka) -------------------------------
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

// ----------------------------- construction ---------------------------------
ManualDrivetrain::ManualDrivetrain(){
    // Ford Transit Mk7 330M RWD: 3300 mm wheelbase (px +-1.65),
    // ~1720 mm track (pz +-0.86), 235/65 R16C wheels (r = 0.345).
    // Front pair free/steering, rear pair driven through an open axle.
    addWheel( 1.65,  0.86, 0.345, false);   // 0 FL
    addWheel( 1.65, -0.86, 0.345, false);   // 1 FR
    addWheel(-1.65,  0.86, 0.345, true );   // 2 RL
    addWheel(-1.65, -0.86, 0.345, true );   // 3 RR
    // front-biased braking, no ABS
    wheels[0].brakeMax = wheels[1].brakeMax = 3500.0;
    wheels[2].brakeMax = wheels[3].brakeMax = 2500.0;
    Differential d; d.a=2; d.b=3; d.mode=DIFF_OPEN;   // stock: open rear diff
    diffs.push_back(d);
    eng.sortCurve();
    publish();
}

void ManualDrivetrain::reset(){
    // reset dynamic state only -- keep the drivetrain configuration
    v=x=accel=0.0;
    eng.omega   = eng.idleRPM*RPM2RAD;
    eng.stalled = false;
    clu.locked  = false;
    clu.lockedish = false;
    box.gear    = 0;                 // start in neutral, engine idling
    for(WheelState& wh : wheels){ wh.omega=wh.kappa=wh.angle=0.0; wh.Fx=0.0; }
    publish();
}

int ManualDrivetrain::drivenCount() const {
    int c=0; for(const WheelState& wh: wheels) if(wh.driven) c++; return c;
}

void ManualDrivetrain::addWheel(double px,double pz,double r,bool driven){
    WheelState wh; wh.px=px; wh.pz=pz; wh.r=r; wh.driven=driven;
    wheels.push_back(wh);
    out_.push_back(WheelOut{});
}

void ManualDrivetrain::removeWheel(int idx){
    if(idx<0 || idx>=(int)wheels.size()) return;
    wheels.erase(wheels.begin()+idx);
    out_.erase(out_.begin()+idx);
    // drop any differential that referenced it; shift higher indices down
    for(int i=(int)diffs.size()-1;i>=0;--i){
        if(diffs[i].a==idx || diffs[i].b==idx){ diffs.erase(diffs.begin()+i); continue; }
        if(diffs[i].a>idx) diffs[i].a--;
        if(diffs[i].b>idx) diffs[i].b--;
    }
}

void ManualDrivetrain::addDiff(){
    Differential d;
    // default to the first two distinct wheels
    d.a = 0;
    d.b = wheels.size()>1 ? 1 : 0;
    diffs.push_back(d);
}

void ManualDrivetrain::removeDiff(int idx){
    if(idx<0 || idx>=(int)diffs.size()) return;
    diffs.erase(diffs.begin()+idx);
}

// ----------------------------- rig description ------------------------------
Layout ManualDrivetrain::layout() const {
    Layout L;
    double minx=0.0,maxx=0.0,minz=0.0,maxz=0.0;
    if(!wheels.empty()){
        minx=maxx=wheels[0].px;
        minz=maxz=wheels[0].pz;
        for(const WheelState& wh: wheels){
            minx=std::min(minx,wh.px); maxx=std::max(maxx,wh.px);
            minz=std::min(minz,wh.pz); maxz=std::max(maxz,wh.pz);
        }
    }
    L.body.mass     = mass;
    L.body.length   = std::max((maxx-minx)+bodyOverhang, 1.2);
    L.body.width    = std::max((maxz-minz)+bodyMargin,   0.8);
    L.body.height   = bodyHeight;
    L.body.comX     = comX;
    L.body.comY     = comY;
    L.body.comZ     = comZ;
    L.body.maxSteer = maxSteer;

    double attachY = -bodyHeight*0.5 + attachAboveFloor;
    L.wheels.reserve(wheels.size());
    for(const WheelState& wh : wheels){
        WheelSpec s;
        s.px        = wh.px;
        s.py        = attachY;
        s.pz        = wh.pz;
        s.radius    = wh.r;
        s.inertia   = wh.I;
        s.brakeMax  = wh.brakeMax;
        s.driven    = wh.driven;
        s.steerable = wh.px > 0.01;    // anything ahead of the centre steers
        L.wheels.push_back(s);
    }
    return L;
}

// ----------------------------- readback -------------------------------------
void ManualDrivetrain::publish(){
    out_.resize(wheels.size());
    for(size_t i=0;i<wheels.size();++i){
        const WheelState& wh = wheels[i];
        out_[i].Fx    = wh.Fx;
        out_[i].Fz    = wh.Fz;
        out_[i].omega = wh.omega;
        out_[i].kappa = wh.kappa;
        out_[i].angle = wh.angle;
        out_[i].steer = wh.steer;
    }
}

Telemetry ManualDrivetrain::telemetry() const {
    Telemetry t;
    t.engineRPM     = eng.omega*RAD2RPM;
    t.idleRPM       = eng.idleRPM;
    t.redline       = eng.redline;
    t.engineStalled = eng.stalled;
    t.clutchLocked  = clu.lockedish;
    t.gear          = box.gear;
    t.gearCount     = box.gears();
    t.gearRatio     = box.gear!=0 ? box.n() : 0.0;
    t.speed         = v;

    int nd = drivenCount();
    t.drivenWheels = nd;
    if(nd>0){
        double c=0.0;
        for(const WheelState& wh: wheels) if(wh.driven) c += wh.omega;
        t.carrierOmega = c/nd;
    }
    return t;
}

// ----------------------------- API entry point ------------------------------
void ManualDrivetrain::step(double dt, const StepInput& in){
    if(wheels.empty()){ v=0; publish(); return; }

    // ---- driver input ------------------------------------------------------
    clu.engagement = clampd(in.cmd.clutch, 0.0, 1.0);
    // no reverse gear is modelled, so the API's -1 lands in neutral
    box.gear = in.cmd.gear <= 0 ? 0 : std::min(in.cmd.gear, box.gears());

    double steerCmd = clampd(in.cmd.steer, -1.0, 1.0);
    for(WheelState& wh : wheels)
        wh.steer = (wh.px > 0.01) ? steerCmd*maxSteer : 0.0;   // fronts steer

    // ---- ground contacts ---------------------------------------------------
    int nc = in.contacts ? std::min(in.contactCount, (int)wheels.size()) : 0;
    for(int i=0;i<nc;++i){
        const ContactIn& c = in.contacts[i];
        wheels[i].grounded = c.grounded;
        wheels[i].Fz       = c.grounded ? c.Fz : 0.0;
        wheels[i].vx       = c.vx;
        wheels[i].mu       = c.mu;
    }
    // corners the host said nothing about fall back to the surface table
    if(nc < (int)wheels.size()){
        double fb = SURFACES[(int)clampd(surf,0,2)].mu;
        for(size_t i=(size_t)nc;i<wheels.size();++i) wheels[i].mu = fb;
    }

    // the external chassis owns the body velocity
    if(external) v = in.bodySpeed;

    // ---- integrate ---------------------------------------------------------
    // The clutch is a stiff coupling, so subdivide the frame here instead of
    // making every host know that.  Capped so one huge hitch cannot stall us.
    int nsub = (int)std::ceil(dt/std::max(1e-6, maxSubstep));
    nsub = std::max(1, std::min(nsub, 64));
    double h = dt/nsub;
    for(int s=0;s<nsub;++s) integrate(h, in.cmd.throttle, in.cmd.brake);

    publish();
}

// ----------------------------- one substep ----------------------------------
void ManualDrivetrain::integrate(double dt,double throttle,double brake){
    int N = (int)wheels.size();
    if(N==0){ v=0; return; }

    // --- weight: distribute static load, then transfer front<->rear on accel
    // (skipped in external mode -- the 3D suspension supplies each wheel's Fz)
    double Wt = mass*g;
    if(!external){
        double staticEach= Wt/N;
        double dFz       = mass*accel*cgH/wheelbase;   // transferred front->rear
        int nf=0,nr=0;
        for(const WheelState& wh: wheels){ if(wh.px>0) nf++; else nr++; }
        for(WheelState& wh: wheels){
            double t = 0.0;
            if(wh.px>0 && nf>0) t = -dFz/nf;           // front sheds load
            else if(wh.px<=0 && nr>0) t = +dFz/nr;     // rear gains load
            wh.Fz = clampd(staticEach + t, 0.0, 1e6);
        }
    }

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
    // Runs before the clutch so the driveline knows the load it is working
    // against when it decides whether the clutch can hold lock.
    std::vector<double> Treact(N,0.0);
    double Fx_total=0.0;
    for(int i=0;i<N;i++){
        WheelState& wh=wheels[i];

        // in external (3D rig) mode each wheel rolls along its own heading over
        // its own patch of ground, so slip is measured against that wheel's
        // contact speed instead of the single body velocity.
        double vref = external ? wh.vx : v;
        double mu   = wh.mu;

        // single-point relaxation model:  sigma*dκ/dt = Vsx - |v|*κ
        //
        // The decay rate is proportional to road speed, so as the car stops the
        // slip state stops relaxing: a wheel locked under braking holds κ ≈ -1
        // (a full skid) straight through zero speed and keeps applying its
        // whole braking force in the same direction, shoving the car backwards
        // and then oscillating.  Flooring the rate at vRelaxMin gives κ a finite
        // time constant (sigma/vRelaxMin) near rest, so below that speed the
        // steady-state slip becomes -v/vRelaxMin: the force fades linearly to
        // zero and the car settles instead of rocking back and forth.
        double Vsx  = wh.omega*wh.r - vref;
        double vdec = std::max(std::fabs(vref), vRelaxMin);
        double dk   = (Vsx - vdec*wh.kappa)/sigma;
        wh.kappa  += dt*dk;
        wh.kappa   = clampd(wh.kappa,-4.0,4.0);

        // an airborne wheel carries no load and makes no force
        double Fpac = (external && !wh.grounded)
                        ? 0.0 : tire.force(wh.kappa, mu, wh.Fz);

        // Standstill damping.  The slip-ratio model relaxes at a rate set by
        // road speed, so as the car stops it runs out of dissipation entirely
        // and the contact rings: the car rocks back and forth under the brakes.
        // Below vStick the contact is modelled directly as a damper resisting
        // sliding, which is what static friction actually does at rest.  The
        // blend is driven by whichever is faster, the road or the tread, so a
        // skid or a wheelspin above walking pace keeps pure Pacejka behaviour
        // and still loses grip as it should.
        //
        // This term deliberately does NOT react back onto wheel spin.  It needs
        // to be stiff enough to damp two and a half tonnes of chassis, and fed
        // through a 1.6 kgm^2 wheel that gain is numerically unstable: the
        // wheels park at the wrong speeds and the axles end up shoving against
        // each other with kilonewtons, which quietly drove the van backwards.
        // At a standstill the patch is stuck and the wheel is held anyway, so
        // the shear goes straight into the body.
        double vTread = std::fabs(wh.omega*wh.r);
        double blend  = 1.0 - clampd(std::max(std::fabs(vref),vTread)/vStick,0.0,1.0);
        double grip   = mu*wh.Fz;
        // Crossfade, don't add.  The relaxed slip ratio behaves like a damper of
        // rate Kstiff/vRelaxMin sitting behind a sigma/vRelaxMin lag, and near
        // standstill that loop gain is enormous: left in, it self-excites and
        // the van rocks on the brakes forever.  Fading it out as the damper
        // fades in removes the lagged path entirely at rest.
        double Fpure  = clampd((1.0-blend)*Fpac, -grip, grip);
        double Fx     = clampd((1.0-blend)*Fpac + stickC*blend*Vsx, -grip, grip);
        bool   flying = external && !wh.grounded;
        wh.Fx     = flying ? 0.0 : Fx;          // what the chassis feels
        Treact[i] = flying ? 0.0 : Fpure*wh.r;  // what spins the wheel
        Fx_total += wh.Fx;
    }

    // --- engine vs driveline coupling through the clutch --------------------
    double n = box.n();
    double omegaCarrier = 0.0;                      // mean driven-wheel speed
    int nd = drivenCount();
    if(nd>0){
        for(const WheelState& wh: wheels) if(wh.driven) omegaCarrier += wh.omega;
        omegaCarrier /= nd;
    }
    double omegaIn = omegaCarrier*n;                // (0 in neutral -> n==0)

    // a stalled (dead) engine produces no combustion torque
    double Te = eng.stalled ? 0.0 : eng.netTorque(throttle);

    double cap = clu.capacity()*clampd(clu.engagement,0.0,1.0);
    bool   path = (box.gear!=0 && nd>0 && std::fabs(n)>1e-9 && cap>1e-9);
    double dW   = eng.omega - omegaIn;

    // Driveline inertia and load, both referred to the crank.  Through a
    // reduction n the wheels look n^2 times lighter, so in 1st the driveline is
    // only ~0.014 kgm^2 against the engine's 0.35: treating the clutch as a
    // stiff spring there is far too fast for the timestep and explodes, which
    // spun the driven wheels backwards and dragged the car around.
    double Jd=0.0, Tref=0.0;
    if(path){
        double Iw=0.0, Tload=0.0;
        for(int i=0;i<N;i++) if(wheels[i].driven){
            Iw    += wheels[i].I;
            Tload += Tlock[i] - Treact[i];
        }
        Jd   = Iw/(n*n*box.eff);
        Tref = Tload/(n*box.eff);
    }

    double Tc = 0.0;
    if(!path){ clu.locked=false; }
    else if(clu.locked){
        // Locked: engine and driveline are one rigid body sharing an
        // acceleration.  Solve for the torque the friction surfaces must carry
        // to hold them together; past their capacity the clutch breaks away.
        double wdot = (Te + Tref)/(eng.I + Jd);
        double Treq = Te - eng.I*wdot;
        if(std::fabs(Treq) <= cap) Tc = Treq;
        else { clu.locked=false; Tc = (Treq>0.0 ? cap : -cap); }
    }
    else Tc = clu.torque(dW);        // slipping: friction torque at the surfaces
    clu.lockedish = clu.locked;

    // engine EoM:  I_e * dω = T_engine - T_clutch
    // No stall floor: if the load (a too-tall gear, clutch dumped at low revs)
    // drags the engine below its stall speed, it dies -- torque cuts out and
    // the car fails to launch, exactly like the real thing.
    eng.omega += dt*(Te - Tc)/eng.I;
    eng.omega  = std::max(eng.omega, 0.0);

    // total axle torque, split equally across the driven wheels (open behaviour)
    double Twheels = Tc*n*box.eff;
    double Tdriven = nd>0 ? Twheels/nd : 0.0;

    // --- per-wheel spin integration -----------------------------------------
    for(int i=0;i<N;i++){
        WheelState& wh=wheels[i];
        double Tdrive = wh.driven ? Tdriven : 0.0;

        // integrate drive + tyre reaction first, then apply the brake as a
        // clamp toward zero: it can LOCK the wheel (omega -> 0, a skid) but can
        // never spin it the other way.  Without this clamp the tyre reaction of
        // a sliding car spins a braked wheel backwards.
        // Rolling resistance (tyre hysteresis) rides along with the brake: both
        // only ever drag the wheel toward a stop, never past it.  Without it a
        // vehicle in neutral free-wheels forever, so the slightest residual
        // force accumulates into a drift.  In standalone 1D mode the chassis
        // model already subtracts Crr, so only charge it once.
        double Troll   = external ? Crr*wh.Fz*wh.r : 0.0;
        double omega2  = wh.omega + dt*(Tdrive + Tlock[i] - Treact[i])/wh.I;
        double dwBrake = (brake*wh.brakeMax + Troll)*dt/wh.I;
        if(omega2 > 0.0) omega2 = std::max(0.0, omega2 - dwBrake);
        else             omega2 = std::min(0.0, omega2 + dwBrake);
        wh.omega = omega2;
        wh.angle += wh.omega*dt;
    }

    // --- clutch lock capture / hold -----------------------------------------
    if(path){
        double c2 = 0.0;
        for(const WheelState& wh: wheels) if(wh.driven) c2 += wh.omega;
        c2 /= nd;
        if(clu.locked){
            // Still locked, so the engine simply goes wherever the driveline
            // went -- including being dragged to a halt by the brakes, which is
            // what stalls the engine when you brake to a stop in gear.
            eng.omega = std::max(0.0, c2*n);
        } else {
            double dW2 = eng.omega - c2*n;
            // surfaces have matched speed -> they grab
            if(dW*dW2 <= 0.0 || std::fabs(dW2) < 0.5){
                clu.locked = true;
                // conserve angular momentum across the engagement; the energy
                // difference is what heats a real clutch up
                double w  = (eng.I*eng.omega + Jd*c2*n)/(eng.I + Jd);
                eng.omega = std::max(0.0, w);
                double shift = w/n - c2;
                for(WheelState& wh: wheels) if(wh.driven) wh.omega += shift;
            }
        }
    }

    // stall check runs after the coupling, so a locked driveline can drag the
    // engine under and kill it
    if(!eng.stalled && eng.omega < eng.stallRPM*RPM2RAD) eng.stalled = true;
    // restart (idle) once the load is removed: neutral or clutch pedal in
    if(eng.stalled && (box.gear==0 || clu.engagement < 0.15)){
        eng.stalled = false;
        eng.omega   = eng.idleRPM*RPM2RAD;
        clu.locked  = false;
    }

    // --- chassis longitudinal dynamics --------------------------------------
    // In external mode the 3D rigid body integrates motion (and feeds v back in
    // through StepInput), so we only run this simple 1D model standalone.
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

// ----------------------------- factory --------------------------------------
std::unique_ptr<IVehicleSim> createTransitDrivetrain(){
    return std::unique_ptr<IVehicleSim>(new ManualDrivetrain());
}

} // namespace vsim
