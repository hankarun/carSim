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

namespace phys {

// ----- suspension settings (one set, shared by all wheels) ------------------
struct Susp {
    float rest      = 0.40f;     // natural suspension length [m]
    float travel    = 0.22f;     // usable compression travel [m]
    float stiffness = 32000.0f;  // spring rate [N/m]
    float damping   = 3800.0f;   // damper rate [N s/m]
    float radius    = 0.42f;     // wheel radius [m]
    float gripK     = 16000.0f;  // lateral grip stiffness [N per m/s]
};

// preconfigured presets the UI can apply
enum SuspPreset { SUSP_SPORT=0, SUSP_COMFORT=1, SUSP_OFFROAD=2, SUSP_COUNT=3 };
Susp        suspPreset(int which);
const char* suspPresetName(int which);

// ----- per-wheel output (world space) ---------------------------------------
struct WheelOut {
    float x=0,y=0,z=0;   // wheel centre, world
    int   grounded=0;
    float Fz=0;          // suspension load [N]
    float compress=0;    // 0 (extended) .. 1 (fully compressed)
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

    // wheel attach offsets in body space (suspension tops), and body box dims
    void buildVehicle(const std::vector<float>& offX,
                      const std::vector<float>& offY,
                      const std::vector<float>& offZ,
                      float bodyLen,float bodyHei,float bodyWid,float mass,
                      float spawnX,float spawnY,float spawnZ,
                      float comX=0,float comY=0,float comZ=0); // COM offset (body space)
    bool hasVehicle() const;
    void setSusp(const Susp& s);
    void resetVehicle(float x,float y,float z);

    // advance one frame.  driveN = longitudinal tire force per wheel [N],
    // steerRad steers the front wheels, mu caps lateral grip.
    void step(float dt, const std::vector<float>& driveN,
              float brake, float steerRad, float mu);

    // read-back ------------------------------------------------------------
    void  bodyPosition(float o[3]) const;     // world position of chassis COM
    void  bodyQuat(float o[4]) const;         // x,y,z,w
    void  bodyDims(float o[3]) const;         // len,hei,wid
    float forwardSpeed() const;               // along body +X [m/s]
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
