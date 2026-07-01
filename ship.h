// =============================================================================
//  ship.h -- simple ship: hull triangle mesh, propeller/rudder, and a custom
//  6-DOF rigid-body integrator with buoyancy computed from triangle centers.
//
//  Body space: +X forward, +Y up, +Z right.
// =============================================================================
#pragma once
#include "raylib.h"
#include <vector>
#include "ocean.h"

struct Tri { Vector3 a,b,c; };               // a hull triangle in body space

// a debug arrow recorded during a step: a world point + the force applied there
// kind: 0 buoyancy, 2 thrust, 3 rudder, 4 hull drag, 5 dry sample point (no force)
struct DbgArrow { Vector3 p; Vector3 f; int kind; };

struct Ship {
    // ---- rigid-body state (custom integrator) ----
    Vector3    pos    = {0,1.0f,0};          // world centre of mass
    Quaternion orient = {0,0,0,1};           // body -> world
    Vector3    vel    = {0,0,0};
    Vector3    angVel = {0,0,0};             // world-space angular velocity (rad/s)

    // ---- mass properties ----
    float mass = 50000.0f;                    // kg (heavy enough to float ~1.1 m draft)
    Vector3 invInertiaDiag = {0,0,0};         // 1/Ixx,1/Iyy,1/Izz (body space)

    // ---- controls ----
    float throttle = 0.0f;                    // -1..1
    float rudder   = 0.0f;                    // -1..1 target
    float rudderAngle = 0.0f;                 // radians (smoothed, for force+render)
    float propOmega = 0.0f;                   // rad/s
    float propAngle = 0.0f;                   // radians (render spin)

    // ---- tuning ----
    float buoyDensity  = 9810.0f;             // rho*g (N per m^2 per m depth)
    float maxDepth     = 4.0f;                // clamp per-facet submergence
    float facetDrag    = 40.0f;               // local drag per facet area
    float heaveDamp    = 120000.0f;           // vertical damping -> rides waves, no bounce
    float linDrag      = 4000.0f;             // forward hull resistance
    float lateralDrag  = 30000.0f;            // sideways (keel) resistance -> carves turns
    float angDrag      = 280000.0f;           // global angular (yaw/roll) drag
    float thrustK      = 120.0f;              // thrust per rad/s of prop
    float maxPropOmega = 220.0f;              // rad/s at full throttle
    float rudderK      = 800.0f;              // rudder plate coeff = 0.5*rho*Cn*area

    // ---- geometry (body space) ----
    std::vector<Tri> hull;                    // used for buoyancy AND rendering
    Vector3 propPos   = {0,0,0};
    Vector3 rudderPos = {0,0,0};
    Vector3 insideRef = {0,0,0};              // a point inside the hull (body space)
    float   hullLen=0, hullBeam=0, hullDepth=0;

    Model model{};                            // hull render model
    bool  built=false;

    // ---- debug ----
    bool  recordDebug=false;                  // when set, fill dbg each step
    std::vector<DbgArrow> dbg;                // sample points + force vectors
};

void shipInit(Ship& s);                       // build hull geometry + mass props
void shipBuildMesh(Ship& s);                  // build raylib model (needs GL ctx)
void shipStep(Ship& s, const Ocean& o, float dt);
void shipReset(Ship& s);
float shipForwardSpeed(const Ship& s);        // signed speed along body +X
void shipUnload(Ship& s);
