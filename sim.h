// =============================================================================
//  sim.h -- vehicle simulation API.
//
//  This header is an INTERFACE, not a model.  It describes the contract between
//  a host (a renderer plus a rigid-body/physics backend) and a vehicle
//  simulation, using nothing but plain data:
//
//      host                                             sim
//      ----                                             ---
//      driver input .................. Command ........>
//      per-wheel ground state ........ ContactIn ......>
//      chassis speed ................. StepInput ......>
//                                                        step(dt, in)
//      <........ per-wheel tyre force  WheelOut ........
//      <........ gauges / readouts     Telemetry ......
//      <........ rig description       Layout .........
//
//  Nothing here knows about Jolt, raylib, or any particular drivetrain.  A host
//  builds its chassis from layout(), feeds ground contacts in, and pushes the
//  resulting longitudinal tyre forces back into its own physics.  That makes
//  three things independently swappable:
//
//    * the SIM      -- manual drivetrain, EV, automatic, tank powertrain, ...
//                      (implement IVehicleSim)
//    * the PHYSICS  -- Jolt, PhysX, Bullet, a custom integrator
//                      (anything that can fill ContactIn and apply WheelOut)
//    * the HOST     -- this raylib front-end, a game engine, a headless test
//
//  Every type crossing the boundary is plain data with no virtual members, so a
//  C ABI or a foreign-language binding can be layered on later without touching
//  any implementation.
//
//  Units are SI throughout: metres, seconds, kilograms, newtons, radians.
//  Body space: +X forward, +Y up, +Z right (matches physics.h).
//
//  The concrete Ford Transit drivetrain lives in drivetrain.h / drivetrain.cpp.
// =============================================================================
#ifndef CARSIM_SIM_H
#define CARSIM_SIM_H

#include <memory>
#include <vector>

// ----------------------------- small helpers --------------------------------
// Deliberately at global scope: the front-ends use these for UI maths too.
double clampd(double v, double lo, double hi);
double sgn(double v);
// piecewise-linear lookup over a monotonically increasing x-table
double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x);

extern const double RAD2RPM;
extern const double RPM2RAD;

// ----------------------------- surfaces -------------------------------------
// A convenience table for hosts that classify ground into a few named types.
// The sim itself never reads this: friction arrives per contact (ContactIn::mu),
// so a vehicle straddling ice and tarmac behaves correctly.
struct Surface { const char* name; double mu; };
extern Surface SURFACES[3];

namespace vsim {

// Bumped whenever anything below changes incompatibly.
constexpr int API_VERSION = 1;

// ============================ INPUT =========================================

// ----------------------------- driver input ---------------------------------
struct Command {
    double throttle = 0.0;   // 0 = closed .. 1 = wide open
    double brake    = 0.0;   // 0 = off .. 1 = full pedal
    double clutch   = 1.0;   // 0 = pedal in (open) .. 1 = fully engaged
    double steer    = 0.0;   // -1 = full left .. +1 = full right (normalized).
                             // The sim maps this onto each steerable wheel and
                             // reports the result in WheelOut::steer.
    int    gear     = 0;     // 0 = neutral, 1..gearCount forward, -1 = reverse.
                             // (An implementation with no reverse gear treats
                             //  negative values as neutral.)
    bool   handbrake = false;
};

// ----------------------------- ground contact -------------------------------
// One per wheel, supplied by the physics backend BEFORE each step.  This is the
// entire picture of the world that the tyre model gets.
struct ContactIn {
    double Fz       = 0.0;   // vertical load carried by this wheel [N]
    double vx       = 0.0;   // ground speed at the contact patch, resolved
                             // along the wheel's own heading [m/s]
    double mu       = 1.0;   // friction coefficient of the ground here
    bool   grounded = true;  // false = airborne: no load, no force
};

// ----------------------------- one step's worth of input --------------------
struct StepInput {
    Command          cmd;                    // driver input
    double           bodySpeed    = 0.0;     // chassis speed along body +X [m/s]
    const ContactIn* contacts     = nullptr; // one per wheel, in layout order
    int              contactCount = 0;       // 0 = no contact data supplied
};

// ============================ OUTPUT ========================================

// ----------------------------- per-wheel result -----------------------------
// Read back after each step.  Fx is what the host must apply to its chassis at
// the contact patch along the wheel's heading; steer is the angle that heading
// sits at.  The rest is for rendering and telemetry.
struct WheelOut {
    double Fx    = 0.0;   // longitudinal tyre force [N]
    double Fz    = 0.0;   // vertical load the sim used [N]
    double omega = 0.0;   // spin speed [rad/s]
    double kappa = 0.0;   // longitudinal slip ratio
    double angle = 0.0;   // accumulated roll angle, for rendering [rad]
    double steer = 0.0;   // steer angle about the suspension axis [rad]
};

// ----------------------------- scalar readouts ------------------------------
struct Telemetry {
    double engineRPM     = 0.0;
    double idleRPM       = 0.0;    // gauge scaling / audio pitch reference
    double redline       = 0.0;
    bool   engineStalled = false;  // engine has died under load
    bool   clutchLocked  = false;  // friction surfaces gripped (not slipping)
    int    gear          = 0;      // same convention as Command::gear
    int    gearCount     = 0;      // number of forward gears
    double gearRatio     = 0.0;    // total reduction, crank -> wheel (0 = neutral)
    double carrierOmega  = 0.0;    // mean driven-wheel speed [rad/s]
    double speed         = 0.0;    // chassis speed along body +X [m/s]
    int    drivenWheels  = 0;
};

// ============================ RIG DESCRIPTION ===============================
// What a host needs in order to BUILD a chassis for this vehicle.  Returned by
// layout(), so a Jolt rig, a PhysX rig or a game-engine prefab can all be
// generated from one description instead of each host hard-coding the geometry.

struct WheelSpec {
    double px        = 0.0;    // suspension-top position, body space [m]
    double py        = 0.0;
    double pz        = 0.0;
    double radius    = 0.345;  // rolling radius [m]
    double inertia   = 1.6;    // wheel + hub inertia [kg m^2]
    double brakeMax  = 3000.0; // brake torque at full pedal [Nm]
    bool   driven    = false;  // fed by the drivetrain
    bool   steerable = false;  // responds to Command::steer
};

struct BodySpec {
    double mass     = 1500.0;  // [kg]
    double length   = 4.0;     // chassis box along X [m]
    double height   = 1.5;     // along Y [m]
    double width    = 1.8;     // along Z [m]
    double comX     = 0.0;     // centre of mass offset from the box centre [m]
    double comY     = 0.0;
    double comZ     = 0.0;
    double maxSteer = 0.63;    // steer angle at full lock [rad]
};

struct Layout {
    BodySpec               body;
    std::vector<WheelSpec> wheels;
};

// ============================ THE INTERFACE =================================
//
//  Per-frame contract:
//     1. fill a StepInput from the physics backend (loads, contact speeds,
//        friction, body speed) and the driver's controls
//     2. step(dt, in)                     -- substepping is the sim's problem
//     3. read wheelOutputs() and push each Fx / steer into the physics
//     4. advance the physics backend
//
//  Implementations must tolerate contactCount != wheelCount() (including 0) by
//  falling back to their own assumptions, so a host can bring the sim up before
//  its chassis exists.
//
class IVehicleSim {
public:
    virtual ~IVehicleSim() = default;

    // human-readable identifier, e.g. "Transit Mk7 330M (manual)"
    virtual const char* name() const = 0;

    // geometry + mass properties the host should build its chassis from
    virtual Layout layout() const = 0;

    // clear all dynamic state; keep the configuration
    virtual void reset() = 0;

    // advance the simulation by dt seconds
    virtual void step(double dt, const StepInput& in) = 0;

    // results, valid until the next step()/reset().  wheelOutputs() points at
    // wheelCount() entries, in the same order as layout().wheels
    virtual int             wheelCount()   const = 0;
    virtual const WheelOut* wheelOutputs() const = 0;
    virtual Telemetry       telemetry()    const = 0;
};

// ----------------------------- factories ------------------------------------
// One per concrete model.  A host that only ever drives the interface needs no
// header other than this one.
std::unique_ptr<IVehicleSim> createTransitDrivetrain();

} // namespace vsim

#endif // CARSIM_SIM_H
