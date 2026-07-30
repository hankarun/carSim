// =============================================================================
//  drivetrain.h -- Ford Transit Mk7 330M manual drivetrain.
//
//  The reference implementation of the vsim::IVehicleSim API (sim.h):
//
//      Engine (editable torque curve) -> multi-plate Clutch -> 6-speed Gearbox
//        -> Final drive -> a configurable set of wheels coupled by
//           Differentials (open / LSD / locked) -> Pacejka tyres -> Chassis
//
//  Everything is data-driven: wheels and differentials live in vectors so a UI
//  can add/remove/edit them (only while the sim is stopped).  Those parts are
//  deliberately left PUBLIC and concrete -- a tuning panel edits a specific
//  drivetrain, not an abstract one.  Hosts that just want to drive the vehicle
//  should include sim.h and go through IVehicleSim instead.
//
//  Only include this header if you need the concrete model.
// =============================================================================
#ifndef CARSIM_DRIVETRAIN_H
#define CARSIM_DRIVETRAIN_H

#include "sim.h"

#include <vector>

namespace vsim {

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
    // index 0 = neutral, then 1st..6th.  No reverse gear is modelled, so the
    // API's gear == -1 is treated as neutral.
    std::vector<double> ratio{ 0.0, 4.21, 2.37, 1.46, 1.00, 0.78, 0.66 };
    double finalDrive = 3.73;     // rear axle crown/pinion 41/11 (Mk7 330M RWD)
    double eff        = 0.92;     // driveline efficiency
    int    gear       = 1;        // 0 = neutral .. gears()

    int    gears() const { return (int)ratio.size()-1; }  // forward gear count
    double n() const { return ratio[gear]*finalDrive; }   // total reduction
};

// ----------------------------- tire (Pacejka) -------------------------------
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
// Both the configuration of a corner and its live state.  The fields fed from
// outside (Fz, vx, grounded, mu) are refreshed from StepInput::contacts at the
// top of every step.
struct WheelState {
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

    // ---- refreshed from the ground contact each step -----------------------
    // Each wheel rolls along its own heading over its own patch of ground, so
    // the slip model needs a per-wheel contact speed and friction rather than
    // one figure for the whole body.
    double vx      = 0.0;    // ground speed at the contact, along the wheel [m/s]
    double mu      = 1.0;    // friction coefficient under this wheel
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

// ----------------------------- load state -----------------------------------
// Named payload steps for the host UI.  These are PAYLOAD only -- the kerb mass
// is a separate parameter, so "laden" means kerb + this.
enum LoadMode { LOAD_EMPTY=0, LOAD_HALF=1, LOAD_LADEN=2, LOAD_OVER=3 };

inline double loadModePayload(int mode){
    switch(mode){
        case LOAD_HALF:  return  575.0;   // half a legal payload
        case LOAD_LADEN: return 1150.0;   // right on the 3500 kg GVM
        case LOAD_OVER:  return 2500.0;   // 2.5 t: way over, and it shows
        default:         return    0.0;   // empty
    }
}

// ----------------------------- the vehicle ----------------------------------
// Ford Transit Mk7 330M (MWB, medium roof), RWD, unladen.
class ManualDrivetrain final : public IVehicleSim {
public:
    // ---- chassis / tyre-contact parameters ---------------------------------
    double mass=2350.0, v=0.0, x=0.0, accel=0.0;   // kerb mass (3500 kg GVM)
    double cgH=0.75, wheelbase=3.30;               // tall body, 3300 mm wheelbase
    double Cd=0.36, A=4.2, rho=1.225, Crr=0.012, g=9.81;   // big flat-fronted box
    double sigma=0.35;        // tire relaxation length [m]
    double vRelaxMin=1.5;     // slip-relaxation speed floor [m/s], see
                              // drivetrain.cpp: stops a locked wheel holding its
                              // braking force through zero speed, rocking the car
    double vStick=0.5;        // below this the contact acts as a damper [m/s].
                              // Keep it small: it must die out well before the
                              // wheels are really rolling, or axles with
                              // different tread speeds get different amounts of
                              // it and fight each other with kilonewtons.
    double stickC=30000.0;    // that damper's rate, per wheel [N per m/s]

    // ---- body box, derived into layout().body -------------------------------
    // Transit MWB: 5.68 m long over a 3.30 m wheelbase (2.38 m of overhang),
    // 1.97 m wide, and a 2.05 m tall box for the medium-roof load bay.
    double bodyOverhang = 2.38;   // added to the wheelbase to get the box length
    double bodyMargin   = 0.25;   // added to the track to get the box width
    double bodyHeight   = 2.05;
    // How far the suspension tops sit above the underside of the box, i.e. the
    // box floor is this far BELOW the attach points -- so smaller = body rides
    // higher.  With the unladen springs settling at 0.32 m the attach points sit
    // 0.667 m up, giving 0.667 - 0.32 = ~0.35 m of clearance: the box floor
    // clears the wheel centres instead of swallowing the wheels to the axle.
    double attachAboveFloor = 0.32;
    // CoM relative to the box centre: forward of centre for the front-mounted
    // engine (~55/45 unladen split) and low enough to sit about 0.74 m above the
    // road -- a rollover threshold of ~1.16 g on a 1.72 m track.
    double comX = 0.16, comY = -0.45, comZ = 0.0;
    double maxSteer = 0.6283185307;   // 36 deg -> ~11.7 m turning circle

    // ---- payload ------------------------------------------------------------
    // `mass` above is the KERB mass, i.e. the empty van.  Cargo is carried as a
    // separate lump rather than folded into `mass`, because the two do not act
    // alike: the load sits low and well behind the kerb CoM, so adding it also
    // moves the weight distribution and the CoM height, not just the total.
    // Everything downstream therefore uses totalMass()/loadedCom*()/loadedCgH()
    // instead of mass/com*/cgH.
    //
    // A 330M's 3500 kg GVM over a 2350 kg kerb leaves 1150 kg of legal payload;
    // LOAD_OVER goes to 2.5 t so an overloaded van can be felt squatting onto
    // its bump stops, going tail-heavy and running out of rear grip.
    double payload  = 0.0;      // cargo in the load bay [kg]
    double payloadX = -0.55;    // its centroid rel. to the box centre [m]:
    double payloadY = -0.55;    // behind the middle, down on the load floor
    double payloadZ =  0.0;

    double totalMass() const { return mass + payload; }
    // mass-weighted blend of a kerb value and a payload value
    double loadBlend(double kerb,double load) const {
        double m = totalMass();
        return m > 1e-9 ? (mass*kerb + payload*load)/m : kerb;
    }
    double loadedComX() const { return loadBlend(comX,payloadX); }
    double loadedComY() const { return loadBlend(comY,payloadY); }
    double loadedComZ() const { return loadBlend(comZ,payloadZ); }
    // CoM height over the road.  comY is measured from the box centre, so the
    // loaded CoM moves by exactly how far the blend shifted it.
    double loadedCgH()  const { return cgH + (loadedComY() - comY); }

    // ---- drivetrain --------------------------------------------------------
    Engine  eng;
    Clutch  clu;
    Gearbox box;
    Tire    tire;

    // Fallback surface index into SURFACES, used only when the host supplies no
    // per-contact friction (standalone / headless runs).
    int     surf=0;

    std::vector<WheelState>   wheels;   // configurable wheel set
    std::vector<Differential> diffs;    // configurable couplings

    // When true the chassis (body velocity v, position x, wheel loads Fz) is
    // owned by an external physics engine: step() then only advances the
    // drivetrain (engine, clutch, gearbox, wheel spin -> tyre Fx) and takes
    // v / Fz from StepInput.  When false it runs its own 1D chassis model.
    bool external = false;

    // The clutch/driveline is a stiff coupling, so step() subdivides dt itself
    // rather than making every host know that.  At 60 Hz this gives 8 substeps,
    // which is what the front-ends used to do by hand.
    double maxSubstep = 1.0/480.0;

    ManualDrivetrain();

    // ---- IVehicleSim -------------------------------------------------------
    const char*     name() const override { return "Transit Mk7 330M (manual)"; }
    Layout          layout() const override;
    void            reset() override;
    void            step(double dt, const StepInput& in) override;
    int             wheelCount()   const override { return (int)wheels.size(); }
    const WheelOut* wheelOutputs() const override { return out_.data(); }
    Telemetry       telemetry()    const override;

    // ---- configuration edits (call only while the sim is stopped) ----------
    void addWheel(double px,double pz,double r,bool driven);
    void removeWheel(int idx);          // also fixes up differential indices
    void addDiff();
    void removeDiff(int idx);
    int  drivenCount() const;

private:
    void integrate(double dt,double throttle,double brake);  // one substep
    void publish();                                          // wheels -> out_

    std::vector<WheelOut> out_;
};

} // namespace vsim

// ---------------------------------------------------------------------------
//  Compatibility aliases.  main.cpp and tank_main.cpp were written against the
//  pre-API names and edit the concrete model directly; keeping these means the
//  API refactor costs them nothing but this include.
// ---------------------------------------------------------------------------
using CurvePoint   = vsim::CurvePoint;
using Engine       = vsim::Engine;
using Clutch       = vsim::Clutch;
using Gearbox      = vsim::Gearbox;
using Tire         = vsim::Tire;
using Wheel        = vsim::WheelState;
using Differential = vsim::Differential;
using DiffMode     = vsim::DiffMode;
using Vehicle      = vsim::ManualDrivetrain;
using vsim::DIFF_OPEN;
using vsim::DIFF_LSD;
using vsim::DIFF_LOCKED;

#endif // CARSIM_DRIVETRAIN_H
