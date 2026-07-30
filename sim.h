// =============================================================================
//  Car drivetrain simulation model (physics only -- no rendering).
//  Engine (editable torque curve) -> multi-plate Clutch -> 3-speed Gearbox ->
//  Final drive -> a configurable set of wheels coupled by Differentials
//  (open / LSD / locked) -> Pacejka tires -> Chassis.
//
//  The drivetrain is data-driven: wheels and differentials live in vectors so
//  the UI can add/remove/edit them (only while the sim is stopped).
// =============================================================================
#ifndef CARSIM_SIM_H
#define CARSIM_SIM_H

#include <vector>

// ----------------------------- small helpers --------------------------------
double clampd(double v, double lo, double hi);
double sgn(double v);
// piecewise-linear lookup over a monotonically increasing x-table
double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x);

extern const double RAD2RPM;
extern const double RPM2RAD;

// ----------------------------- engine ---------------------------------------
struct CurvePoint { double rpm, nm; };

// Ford Transit Mk7 2.2 TDCi Duratorq, 125 PS (92 kW @ 3500 rpm), 350 Nm
// @ 1450-2000 rpm.  Diesel: heavy dual-mass flywheel, low idle, 4300 rpm cut.
struct Engine {
    double I       = 0.35;      // flywheel inertia [kg m^2] (dual-mass)
    double omega   = 84.0;      // [rad/s]  (~800 rpm idle)
    double idleRPM = 800.0;
    double redline = 4300.0;    // fuel cut / governor
    double stallRPM= 400.0;
    bool   stalled = false;     // engine has died (dragged below stall under load)

    // wide-open-throttle torque curve, kept sorted by rpm (editable).
    // Flat 350 Nm shelf from 1450-2000 rpm, 92 kW peak at 3500 rpm.
    std::vector<CurvePoint> curve{
        {800,180},{1200,280},{1450,350},{2000,350},{2500,320},
        {3000,285},{3500,250},{4000,200},{4300,150} };

    void   sortCurve();                 // keep points monotonic in rpm
    double torqueAt(double rpm) const;  // interpolate the curve
    double netTorque(double throttle) const;
};

// ----------------------------- clutch ---------------------------------------
// Soft-locking multi-plate friction clutch: each plate adds capacity; a tanh
// smoothly "locks" the pack as the speed difference -> 0 (no chatter).
struct Clutch {
    // Transit: a single heavy dry plate rated ~1.3x the engine's peak torque.
    double capacityPerPlate = 460.0; // torque capacity of one friction plate [Nm]
    int    plates     = 1;           // number of clutch plates (the "clutch count")
    double engagement = 1.0;         // 0 = pedal in (open), 1 = fully engaged
    double band       = 5.0;         // slip-speed scale for the soft lock [rad/s]
    bool   lockedish  = false;       // display flag
    bool   locked     = false;       // surfaces gripped: engine + driveline rigid

    double capacity() const { return capacityPerPlate * (double)plates; }
    double torque(double dOmega) const;
};

// ----------------------------- gearbox --------------------------------------
struct Gearbox {
    // Ford VMT6 six-speed manual: direct 4th plus two overdrives.
    // index 0 = neutral, then 1st..6th.
    std::vector<double> ratio{ 0.0, 4.21, 2.37, 1.46, 1.00, 0.78, 0.66 };
    double finalDrive = 3.73;     // rear axle crown/pinion 41/11 (Mk7 330M RWD)
    double eff        = 0.92;     // driveline efficiency
    int    gear       = 1;        // 0 = neutral .. gears()

    int    gears() const { return (int)ratio.size()-1; }  // forward gear count
    double n() const { return ratio[gear]*finalDrive; }   // total reduction
};

// ----------------------------- tire (Pacejka) -------------------------------
struct Surface { const char* name; double mu; };
extern Surface SURFACES[3];

struct Tire {
    // shape coefficients (carcass) held fixed; D and B rebuilt from mu, Fz.
    double C = 1.60;
    double E = 0.97;
    double Kstiff = 100000.0;  // longitudinal slip stiffness B*C*D (held fixed)
                               // 235/65 R16C load-rated van carcass

    double D(double mu,double Fz)  const;
    double B(double mu,double Fz)  const;
    double kappaPeak(double mu,double Fz) const;
    double force(double kappa,double mu,double Fz) const;
};

// ----------------------------- wheel ----------------------------------------
struct Wheel {
    double I      = 1.6;     // wheel+hub inertia [kg m^2] (16" van wheel)
    double r      = 0.345;   // rolling radius [m] (235/65 R16C)
    double omega  = 0.0;     // spin speed [rad/s]
    double kappa  = 0.0;     // slip ratio (relaxation state)
    double angle  = 0.0;     // visual spin angle
    bool   driven = false;
    double Fx     = 0.0;     // last longitudinal force (for display)
    double Fz     = 5765.0;  // vertical load (2350 kg kerb / 4 corners)
    double px     = 0.0;     // longitudinal position (forward = +) [m]
    double pz     = 0.0;     // lateral position (left = +) [m]

    // ---- external (3D rig) mode only --------------------------------------
    // Each wheel rolls along its own heading over its own patch of ground, so
    // the slip model needs a per-wheel contact speed rather than the body's.
    double vx      = 0.0;    // ground speed at the contact, along the wheel [m/s]
    double steer   = 0.0;    // steer angle about the suspension axis [rad]
    bool   grounded= true;   // airborne wheels make no force
    double brakeMax= 3000.0; // brake torque at full pedal [Nm]
};

// ----------------------------- differential ---------------------------------
// Couples two wheels.  OPEN = no coupling (equal torque, free speed split),
// LSD = limited-slip (caps the transferable locking torque), LOCKED = strong
// coupling that drags the two wheel speeds together.
enum DiffMode { DIFF_OPEN=0, DIFF_LSD=1, DIFF_LOCKED=2 };

struct Differential {
    int      a = 0, b = 1;    // wheel indices it joins
    DiffMode mode = DIFF_OPEN;
    double   lockCap = 300.0; // LSD locking-torque cap [Nm] (LOCKED uses 5x)
    double   band    = 4.0;   // slip-speed scale for the soft lock [rad/s]

    // effective locking-torque capacity given the mode
    double effectiveCap() const {
        if(mode==DIFF_OPEN)   return 0.0;
        if(mode==DIFF_LOCKED) return lockCap*5.0;
        return lockCap;
    }
};

// ----------------------------- vehicle --------------------------------------
// Ford Transit Mk7 330M (MWB, medium roof), RWD, unladen.
struct Vehicle {
    double mass=2350.0, v=0.0, x=0.0, accel=0.0;   // kerb mass (3500 kg GVM)
    double cgH=0.75, wheelbase=3.30;               // tall body, 3300 mm wheelbase
    double Cd=0.36, A=4.2, rho=1.225, Crr=0.012, g=9.81;   // big flat-fronted box
    double sigma=0.35;        // tire relaxation length [m]
    double vRelaxMin=1.5;     // slip-relaxation speed floor [m/s], see sim.cpp:
                              // stops a locked wheel holding its braking force
                              // through zero speed and rocking the car
    double vStick=0.5;        // below this the contact acts as a damper [m/s].
                              // Keep it small: it must die out well before the
                              // wheels are really rolling, or axles with
                              // different tread speeds get different amounts of
                              // it and fight each other with kilonewtons.
    double stickC=30000.0;    // that damper's rate, per wheel [N per m/s]

    Engine  eng;
    Clutch  clu;
    Gearbox box;
    Tire    tire;
    int     surf=0;

    std::vector<Wheel>        wheels;   // configurable wheel set
    std::vector<Differential> diffs;    // configurable couplings

    // When true the chassis (body velocity v, position x, wheel loads Fz) is
    // owned by the external 3D physics engine: step() then only advances the
    // drivetrain (engine, clutch, gearbox, wheel spin -> tire Fx) and leaves
    // v / Fz to be supplied from outside before each call.
    bool external = false;

    Vehicle();
    void reset();                       // reset dynamic state, keep config
    void step(double dt,double throttle,double brake);

    // ---- configuration edits (call only while the sim is stopped) ----------
    void addWheel(double px,double pz,double r,bool driven);
    void removeWheel(int idx);          // also fixes up differential indices
    void addDiff();
    void removeDiff(int idx);
    int  drivenCount() const;
};

#endif // CARSIM_SIM_H
