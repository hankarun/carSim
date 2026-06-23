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

struct Engine {
    double I       = 0.22;      // flywheel inertia [kg m^2]
    double omega   = 84.0;      // [rad/s]  (~800 rpm idle)
    double idleRPM = 800.0;
    double redline = 6500.0;
    double stallRPM= 350.0;
    bool   stalled = false;     // engine has died (dragged below stall under load)

    // wide-open-throttle torque curve, kept sorted by rpm (editable)
    std::vector<CurvePoint> curve{
        {800,150},{1500,200},{2500,235},{3500,250},
        {4500,248},{5500,225},{6200,195},{6500,175} };

    void   sortCurve();                 // keep points monotonic in rpm
    double torqueAt(double rpm) const;  // interpolate the curve
    double netTorque(double throttle) const;
};

// ----------------------------- clutch ---------------------------------------
// Soft-locking multi-plate friction clutch: each plate adds capacity; a tanh
// smoothly "locks" the pack as the speed difference -> 0 (no chatter).
struct Clutch {
    double capacityPerPlate = 235.0; // torque capacity of one friction plate [Nm]
    int    plates     = 2;           // number of clutch plates (the "clutch count")
    double engagement = 1.0;         // 0 = pedal in (open), 1 = fully engaged
    double band       = 6.0;         // slip-speed scale for the soft lock [rad/s]
    bool   lockedish  = false;       // display flag

    double capacity() const { return capacityPerPlate * (double)plates; }
    double torque(double dOmega) const;
};

// ----------------------------- gearbox --------------------------------------
struct Gearbox {
    // index 0 = neutral. ratios for 1st..3rd.
    std::vector<double> ratio{ 0.0, 3.20, 1.90, 1.30 };  // [0]=neutral, then gears
    double finalDrive = 3.90;
    double eff        = 0.90;     // driveline efficiency
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
    double Kstiff = 78000.0;   // longitudinal slip stiffness B*C*D (held fixed)

    double D(double mu,double Fz)  const;
    double B(double mu,double Fz)  const;
    double kappaPeak(double mu,double Fz) const;
    double force(double kappa,double mu,double Fz) const;
};

// ----------------------------- wheel ----------------------------------------
struct Wheel {
    double I      = 0.9;     // wheel+hub inertia [kg m^2]
    double r      = 0.31;    // rolling radius [m]
    double omega  = 0.0;     // spin speed [rad/s]
    double kappa  = 0.0;     // slip ratio (relaxation state)
    double angle  = 0.0;     // visual spin angle
    bool   driven = false;
    double Fx     = 0.0;     // last longitudinal force (for display)
    double Fz     = 3200.0;  // vertical load
    double px     = 0.0;     // longitudinal position (forward = +) [m]
    double pz     = 0.0;     // lateral position (left = +) [m]
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
struct Vehicle {
    double mass=1300.0, v=0.0, x=0.0, accel=0.0;
    double cgH=0.52, wheelbase=2.60;
    double Cd=0.32, A=2.2, rho=1.225, Crr=0.013, g=9.81;
    double sigma=0.35;        // tire relaxation length [m]

    Engine  eng;
    Clutch  clu;
    Gearbox box;
    Tire    tire;
    int     surf=0;

    std::vector<Wheel>        wheels;   // configurable wheel set
    std::vector<Differential> diffs;    // configurable couplings

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
