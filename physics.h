// =============================================================================
//  Jolt-backed 3D vehicle physics (raycast suspension on a heightmap).
//
//  This header is deliberately Jolt-free: all Jolt types are hidden behind a
//  pimpl so the rest of the program (main.cpp, sim.cpp) needs no Jolt headers.
//
//  Coordinate convention (body space):  +X forward, +Y up, +Z right.
// =============================================================================
#ifndef CARSIM_PHYSICS_H
#define CARSIM_PHYSICS_H

#include <vector>
#include <cfloat>

namespace phys {

// ----- suspension settings (one set, shared by all wheels) ------------------
// Defaults: Ford Transit Mk7 330M, unladen (coil front / leaf rear compromise).
struct Susp {
    float rest      = 0.45f;     // natural suspension length [m]
    float travel    = 0.20f;     // usable compression travel [m]
    float stiffness = 45000.0f;  // spring rate [N/m]
    float damping   = 5600.0f;   // damper rate [N s/m]
    float radius    = 0.345f;    // wheel radius [m] (235/65 R16C)
    float gripK     = 24000.0f;  // lateral grip stiffness [N per m/s]
};

// preconfigured presets the UI can apply (van load states)
enum SuspPreset { SUSP_UNLADEN=0, SUSP_LADEN=1, SUSP_ROUGH=2, SUSP_COUNT=3 };
Susp        suspPreset(int which);
const char* suspPresetName(int which);

// ----- tank track settings (one set, shared by all track rays) --------------
// The track is modelled as two longitudinal rows of downward rays (left/right).
// Each ray is a small tyre: spring+damper load, a longitudinal thrust from
// track-vs-ground slip, and a lateral grip force -- all capped by mu*Fz.
struct TrackSusp {
    float rest      = 0.55f;     // natural suspension length [m]
    float travel    = 0.30f;     // usable compression travel [m]
    float stiffness = 900000.0f; // spring rate [N/m]  (heavy tank)
    float damping   = 120000.0f; // damper rate [N s/m]
    float radius    = 0.35f;     // road-wheel radius [m]
    float gripK     = 60000.0f;  // lateral grip stiffness [N per m/s]
    float trackK    = 250000.0f; // longitudinal slip stiffness [N per m/s]
                                 //   HIGH -> tiny slip carries the load -> the
                                 //   track "sticks" to the ground (near no-slip)
};

// ----- per-track aggregate readback (side 0 = left, 1 = right) --------------
struct TrackOut {
    float force        = 0;  // summed longitudinal reaction along body +X [N]
    float load         = 0;  // summed suspension load Fz [N]
    int   groundedRays = 0;  // how many of this track's rays touched ground
    float contactSpeed = 0;  // mean along-forward ground speed at contacts [m/s]
};

// ----- per-wheel output (world space) ---------------------------------------
struct WheelOut {
    float x=0,y=0,z=0;   // wheel centre, world
    int   grounded=0;
    float Fz=0;          // suspension load [N]
    float compress=0;    // 0 (extended) .. 1 (fully compressed)
    float steer=0;       // steer angle about the suspension axis [rad]
    float spin=0;        // accumulated rolling angle [rad]
    float slip=0;        // longitudinal slip ratio (0 = rolling, 1 = locked/spin)
    float Fx=0;          // longitudinal tyre force this step [N]
    float omega=0;       // wheel spin rate [rad/s]
    // rig mode: ground velocity at the contact patch, in wheel axes
    float vx=0;          // along the wheel's heading [m/s]
    float vy=0;          // sideways (positive = sliding right) [m/s]
};

// ----- raycast rig input (one per wheel, supplied by the caller's drivetrain)
// In rig mode Jolt only owns the chassis body, the raycasts and the collision;
// the engine / clutch / gearbox / differential all live outside, so the caller
// hands back the longitudinal tyre force each wheel is making.
struct RigWheelIn {
    float steer = 0.0f;  // steer angle about the suspension axis [rad]
    float Fx    = 0.0f;  // longitudinal tyre force along the wheel [N]
};

// ----- drivetrain configuration for the Jolt WheeledVehicleController --------
// Maps the editable engine / gearbox / final-drive spec onto Jolt's built-in
// engine + transmission + differential.  Built once per (re)spawn.
struct Drivetrain {
    // engine -- 2.2 TDCi Duratorq 125 PS ------------------------------------
    float maxTorque = 350.0f;         // peak of the torque curve [Nm]
    float minRPM    = 800.0f;         // idle / stall floor [rpm]
    float maxRPM    = 4300.0f;        // redline [rpm]
    float inertia   = 0.35f;          // flywheel inertia [kg m^2]
    std::vector<float> curveRPM;      // torque-curve X (rpm), ascending
    std::vector<float> curveNm;       // torque-curve Y (Nm)

    // transmission -- Ford VMT6 six-speed manual -----------------------------
    std::vector<float> gearRatios{4.21f,2.37f,1.46f,1.00f,0.78f,0.66f};
    float reverseRatio = -3.84f;      // reverse gear
    float finalDrive = 3.73f;         // rear axle ratio (Mk7 330M RWD, 41/11)
    float lsdRatio   = FLT_MAX;       // per-diff limited-slip ratio (FLT_MAX=open)
    float clutchStrength = 12.0f;     // heavy single dry plate
    float shiftTime  = 0.45f;         // gate-to-gate shift time [s]
    float stallRPM   = 400.0f;        // dragged below this in gear -> engine dies
};

// ----- obstacle (ramp / crate) world transform for rendering ----------------
struct ObstacleOut {
    int   kind=0;        // 0 = crate (box), 1 = ramp (wedge)
    float px=0,py=0,pz=0;
    float qx=0,qy=0,qz=0,qw=1;
    float sx=1,sy=1,sz=1; // crate: half-extents ; ramp: (length,height,width)
    int   dynamic=0;
};

class World {
public:
    World();
    ~World();
    World(const World&)            = delete;
    World& operator=(const World&) = delete;

    // terrain: n x n height samples on a square grid (cell metres apart),
    // grid centred on the origin.  Call once before buildVehicle().
    void setHeightfield(int n, float cell, const std::vector<float>& heights);
    float terrainCell()   const;
    int   terrainN()      const;
    float heightSample(int i,int j) const;   // raw sample (clamped indices)

    // wheel attach offsets in body space (suspension tops), and body box dims.
    // driven[i]!=0 marks a powered wheel; wheels with offX>0 also steer.
    // The Jolt WheeledVehicleController owns the engine/gearbox/diff described
    // by `dt`.
    void buildVehicle(const std::vector<float>& offX,
                      const std::vector<float>& offY,
                      const std::vector<float>& offZ,
                      const std::vector<int>&   driven,
                      const Drivetrain& dt,
                      float bodyLen,float bodyHei,float bodyWid,float mass,
                      float spawnX,float spawnY,float spawnZ,
                      float comX=0,float comY=0,float comZ=0); // COM offset (body space)
    bool hasVehicle() const;
    void setSusp(const Susp& s);
    void resetVehicle(float x,float y,float z);

    // ----- tracked tank ---------------------------------------------------
    // Build a tank chassis with two rows of road-wheel rays (left/right track).
    // roadWheelsPerSide rays are spread evenly along X in [x0,x1] at height
    // attachY; the left row sits at Z=leftZ, the right row at Z=rightZ.
    // wheels() is sized to 2*roadWheelsPerSide (left row first, then right).
    void buildTank(int roadWheelsPerSide, float x0, float x1,
                   float leftZ, float rightZ, float attachY,
                   float bodyLen, float bodyHei, float bodyWid, float mass,
                   float spawnX, float spawnY, float spawnZ,
                   float comX=0, float comY=0, float comZ=0);
    void setTrackSusp(const TrackSusp& s);

    // ----- raycast car rig (no Jolt vehicle controller) -------------------
    // Same chassis + wheel layout as buildVehicle(), but nothing drives the
    // wheels: Jolt just provides the rigid body, the suspension raycasts and
    // the collision.  The caller runs its own drivetrain, reads each wheel's
    // load and contact speed back from wheels(), and feeds the resulting
    // longitudinal tyre forces into stepRig().
    void buildRig(const std::vector<float>& offX,
                  const std::vector<float>& offY,
                  const std::vector<float>& offZ,
                  float bodyLen,float bodyHei,float bodyWid,float mass,
                  float spawnX,float spawnY,float spawnZ,
                  float comX=0,float comY=0,float comZ=0);

    // advance one frame of the raycast rig.  `in` is one entry per wheel;
    // mu caps every contact force by the friction circle.
    void stepRig(float dt, const std::vector<RigWheelIn>& in, float mu);

    // advance one frame in tank mode.  leftSurf/rightSurf are the commanded
    // TRACK SURFACE SPEEDS [m/s] (sprocket omega * radius) of each track;
    // brake in [0,1] holds the tank; mu caps longitudinal & lateral grip.
    void stepTank(float dt, float leftSurf, float rightSurf,
                  float brake, float mu);
    const TrackOut& track(int side) const;   // side 0 = left, 1 = right

    // advance one frame with the Jolt vehicle controller.
    //   throttle  [0,1]   accelerator
    //   brake     [0,1]   foot brake
    //   steer     [-1,1]  normalized steering (left .. right)
    //   gear      0=N, 1..=forward, -1=reverse
    //   clutch    [0,1]   1 = fully engaged
    //   mu                surface grip applied to the ground contact
    void step(float dt, float throttle, float brake, float steer,
              int gear, float clutch, float mu);

    // read-back ------------------------------------------------------------
    void  bodyPosition(float o[3]) const;     // world position of chassis COM
    void  bodyQuat(float o[4]) const;         // x,y,z,w
    void  bodyDims(float o[3]) const;         // len,hei,wid
    float forwardSpeed() const;               // along body +X [m/s]
    float engineRPM() const;                  // Jolt engine speed [rpm], 0 if stalled
    bool  engineStalled() const;              // engine dragged below stall speed
    int   currentGear() const;                // Jolt transmission gear
    const std::vector<WheelOut>& wheels() const { return wout_; }

    // obstacles ------------------------------------------------------------
    void addCrate(float x,float y,float z,float half,float mass);
    void addRamp (float x,float y,float z,float yawRad,
                  float length,float height,float width);
    void resetObstacles();                    // dynamic crates back to spawn
    const std::vector<ObstacleOut>& obstacles() const { return obs_; }

private:
    struct Impl;
    Impl* p_;
    std::vector<WheelOut>    wout_;
    std::vector<ObstacleOut> obs_;
};

} // namespace phys

#endif // CARSIM_PHYSICS_H
