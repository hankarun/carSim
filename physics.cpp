// =============================================================================
//  Jolt-backed 3D vehicle physics -- implementation.
//
//  A single rigid body is the chassis.  Each wheel casts a ray straight down in
//  body space; where it hits the heightfield we apply a spring+damper
//  suspension force, a longitudinal tire force (from the drivetrain) and a
//  lateral grip force.  The body is otherwise free to pitch/roll/yaw, so it
//  tips over crests and leans under load -- a classic "raycast vehicle".
// =============================================================================
#include "physics.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>

#include <thread>
#include <algorithm>
#include <cmath>

JPH_SUPPRESS_WARNINGS

using namespace JPH;
using namespace JPH::literals;

// ----------------------------- collision layers -----------------------------
namespace Layers {
    static constexpr ObjectLayer NON_MOVING = 0;
    static constexpr ObjectLayer MOVING     = 1;
    static constexpr ObjectLayer NUM_LAYERS = 2;
}
namespace BroadPhaseLayers {
    static constexpr BroadPhaseLayer NON_MOVING(0);
    static constexpr BroadPhaseLayer MOVING(1);
    static constexpr uint NUM_LAYERS = 2;
}

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl(){
        mMap[Layers::NON_MOVING]=BroadPhaseLayers::NON_MOVING;
        mMap[Layers::MOVING]    =BroadPhaseLayers::MOVING;
    }
    uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer l) const override { return mMap[l]; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override { return "layer"; }
#endif
private:
    BroadPhaseLayer mMap[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(ObjectLayer o1, BroadPhaseLayer o2) const override {
        if(o1==Layers::NON_MOVING) return o2==BroadPhaseLayers::MOVING;
        return true;
    }
};
class ObjectLayerPairFilterImpl final : public ObjectLayerPairFilter {
public:
    bool ShouldCollide(ObjectLayer o1, ObjectLayer o2) const override {
        if(o1==Layers::NON_MOVING) return o2==Layers::MOVING;
        return true;
    }
};
// ray filter: only the static ground layer (so casts ignore the chassis itself)
class GroundOnlyLayerFilter final : public ObjectLayerFilter {
public:
    bool ShouldCollide(ObjectLayer l) const override { return l==Layers::NON_MOVING; }
};

namespace phys {

// ----------------------------- presets --------------------------------------
// Ford Transit Mk7 330M load states.  Fields:
//   rest, travel, stiffness, damping, radius, gripK
Susp suspPreset(int which){
    Susp s;
    switch(which){
        case SUSP_LADEN:   s={0.45f,0.24f,62000.f,7200.f,0.345f,30000.f}; break;
        case SUSP_ROUGH:   s={0.52f,0.32f,38000.f,5200.f,0.345f,20000.f}; break;
        case SUSP_UNLADEN:
        default:           s={0.45f,0.20f,45000.f,5600.f,0.345f,24000.f}; break;
    }
    return s;
}
const char* suspPresetName(int which){
    switch(which){ case SUSP_LADEN: return "LADEN";
                   case SUSP_ROUGH: return "ROUGH";
                   default:         return "UNLADEN"; }
}

// ----------------------------- pimpl ----------------------------------------
struct World::Impl {
    TempAllocatorImpl*      temp = nullptr;
    JobSystemThreadPool*    jobs = nullptr;
    PhysicsSystem           sys;
    BPLayerInterfaceImpl            bpli;
    ObjectVsBroadPhaseLayerFilterImpl ovbp;
    ObjectLayerPairFilterImpl         ovo;
    GroundOnlyLayerFilter           groundFilter;

    BodyID ground;
    BodyID chassis;
    bool   haveChassis = false;

    std::vector<BodyID>      obsId;     // obstacle bodies
    std::vector<ObstacleOut> obsInit;   // spawn transforms (for reset)

    // terrain
    int   hmN    = 0;
    float hmCell = 2.0f;
    std::vector<float> heights;

    // wheels (body-space attach offsets)
    std::vector<float> ox, oy, oz;
    std::vector<Vec3>  brakeAnchor;  // per-wheel static-brake stick point (world)
    std::vector<char>  brakeStuck;   // 1 = wheel is currently holding statically
    float bodyLen=4, bodyHei=0.8f, bodyWid=1.8f, mass=1300;
    Susp susp;

    // Jolt vehicle (car mode): constraint owns wheels + WheeledVehicleController
    Ref<VehicleConstraint>     vehicle;
    Ref<VehicleCollisionTester> tester;
    std::vector<float>         spin;      // per-wheel accumulated rolling angle
    bool                       isCar = false;

    // stall model (Jolt's engine clamps at mMinRPM and can never die)
    std::vector<int> drivenIdx;      // wheels fed by the differentials
    float finalDrive = 3.73f;        // differential ratio
    float stallRPM   = 400.0f;       // engine dies below this
    bool  stalled    = false;
    float stallT     = 0.0f;         // how long we've been below stall speed [s]

    // water / buoyancy
    Buoy               buoy;
    std::vector<Vec3>  buoyLocal;  // sample points, body space
    float              buoySlabH = 1.0f;  // vertical extent one point stands for [m]

    // tank track state
    std::vector<char> trackSide;   // per ray: 0 = left track, 1 = right track
    TrackSusp         tsusp;
    TrackOut          tout[2];     // per-track aggregate readback (0=L,1=R)
    bool              isTank = false;
    bool              isRig  = false;  // custom raycast car (no Jolt controller)
};

World::World() {
    p_ = new Impl();
    RegisterDefaultAllocator();
    if(Factory::sInstance==nullptr) Factory::sInstance = new Factory();
    RegisterTypes();
    p_->temp = new TempAllocatorImpl(24*1024*1024);
    p_->jobs = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers,
                  std::max(1u, std::thread::hardware_concurrency()-1));
    p_->sys.Init(1024, 0, 2048, 2048, p_->bpli, p_->ovbp, p_->ovo);
    p_->sys.SetGravity(Vec3(0,-9.81f,0));
}

World::~World() {
    BodyInterface& bi = p_->sys.GetBodyInterface();
    if(p_->vehicle!=nullptr){
        p_->sys.RemoveStepListener(p_->vehicle);
        p_->sys.RemoveConstraint(p_->vehicle);
        p_->vehicle=nullptr;
    }
    for(BodyID id : p_->obsId){ bi.RemoveBody(id); bi.DestroyBody(id); }
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis); }
    if(!p_->ground.IsInvalid()){ bi.RemoveBody(p_->ground); bi.DestroyBody(p_->ground); }
    delete p_->jobs;
    delete p_->temp;
    delete p_;
    // leave Factory / RegisterTypes in place (single world per process)
}

// ----------------------------- terrain --------------------------------------
void World::setHeightfield(int n, float cell, const std::vector<float>& heights){
    p_->hmN=n; p_->hmCell=cell; p_->heights=heights;

    BodyInterface& bi = p_->sys.GetBodyInterface();
    if(!p_->ground.IsInvalid()){ bi.RemoveBody(p_->ground); bi.DestroyBody(p_->ground); }

    // centre the grid on the origin
    float span = (n-1)*cell;
    Vec3 offset(-span*0.5f, 0.0f, -span*0.5f);
    Vec3 scale (cell, 1.0f, cell);
    HeightFieldShapeSettings hs(heights.data(), offset, scale, (uint32)n);
    ShapeSettings::ShapeResult res = hs.Create();
    Body* g = bi.CreateBody(BodyCreationSettings(res.Get(),
                RVec3(0,0,0), Quat::sIdentity(),
                EMotionType::Static, Layers::NON_MOVING));
    p_->ground = g->GetID();
    bi.AddBody(p_->ground, EActivation::DontActivate);
}
float World::terrainCell() const { return p_->hmCell; }
int   World::terrainN()    const { return p_->hmN; }
float World::heightSample(int i,int j) const {
    if(p_->hmN<=0) return 0.0f;
    i=std::max(0,std::min(p_->hmN-1,i));
    j=std::max(0,std::min(p_->hmN-1,j));
    return p_->heights[(size_t)i*p_->hmN + j];
}

// ----------------------------- vehicle --------------------------------------
void World::buildVehicle(const std::vector<float>& offX,
                         const std::vector<float>& offY,
                         const std::vector<float>& offZ,
                         const std::vector<int>&   driven,
                         const Drivetrain& dt,
                         float bodyLen,float bodyHei,float bodyWid,float mass,
                         float spawnX,float spawnY,float spawnZ,
                         float comX,float comY,float comZ){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    // tear down any previous vehicle constraint before its body goes away
    if(p_->vehicle!=nullptr){
        p_->sys.RemoveStepListener(p_->vehicle);
        p_->sys.RemoveConstraint(p_->vehicle);
        p_->vehicle=nullptr;
    }
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis);
                         p_->haveChassis=false; }

    size_t nW = offX.size();
    p_->ox=offX; p_->oy=offY; p_->oz=offZ;
    p_->spin.assign(nW, 0.0f);
    p_->bodyLen=bodyLen; p_->bodyHei=bodyHei; p_->bodyWid=bodyWid; p_->mass=mass;
    p_->isCar = true;
    wout_.assign(nW, WheelOut{});

    // box chassis, wrapped so its centre of mass can be shifted (body space)
    BoxShapeSettings boxS(Vec3(bodyLen*0.5f, bodyHei*0.5f, bodyWid*0.5f));
    ShapeRefC boxShape = boxS.Create().Get();
    OffsetCenterOfMassShapeSettings comS(Vec3(comX,comY,comZ), boxShape);
    ShapeSettings::ShapeResult res = comS.Create();
    BodyCreationSettings bcs(res.Get(),
        RVec3(spawnX,spawnY,spawnZ), Quat::sIdentity(),
        EMotionType::Dynamic, Layers::MOVING);
    bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = mass;
    bcs.mLinearDamping  = 0.05f;
    bcs.mAngularDamping = 0.20f;
    Body* b = bi.CreateBody(bcs);
    p_->chassis = b->GetID();
    bi.AddBody(p_->chassis, EActivation::Activate);
    p_->haveChassis = true;

    // ---- assemble the Jolt vehicle constraint ------------------------------
    const Susp& s = p_->susp;
    // convert the physical spring (k [N/m], c [N s/m]) into Jolt's
    // frequency+damping-ratio form using a per-corner sprung mass so the
    // SPORT/COMFORT/OFFROAD presets keep meaning.
    float cornerM = std::max(1.0f, mass / std::max<size_t>(1,nW));
    float wn      = std::sqrt(std::max(1.0f, s.stiffness) / cornerM);   // rad/s
    float freq    = std::max(0.6f, std::min(4.0f, wn/(2.0f*JPH_PI)));   // Hz
    float zeta    = s.damping / (2.0f*std::sqrt(std::max(1.0f,s.stiffness)*cornerM));
    zeta          = std::max(0.1f, std::min(1.2f, zeta));

    VehicleConstraintSettings vs;
    vs.mUp      = Vec3(0,1,0);
    vs.mForward = Vec3(1,0,0);          // our body convention: +X forward
    vs.mMaxPitchRollAngle = DegreesToRadians(60.0f);

    const float MAX_STEER = DegreesToRadians(36.0f);   // ~11.7 m turning circle
    for(size_t i=0;i<nW;++i){
        WheelSettingsWV* w = new WheelSettingsWV();
        // attach point: offY is the suspension top; drop by the rest length so
        // the wheel sits where the old raycast model put it.
        w->mPosition            = Vec3(offX[i], offY[i], offZ[i]);
        w->mSuspensionDirection = Vec3(0,-1,0);
        w->mSteeringAxis        = Vec3(0,1,0);
        w->mWheelUp             = Vec3(0,1,0);
        w->mWheelForward        = Vec3(1,0,0);
        w->mRadius              = s.radius;
        w->mWidth               = 0.235f;              // 235-section van tyre
        w->mSuspensionMinLength = std::max(0.0f, s.rest - s.travel);
        w->mSuspensionMaxLength = s.rest;
        w->mSuspensionSpring    = SpringSettings(ESpringMode::FrequencyAndDamping,
                                                 freq, zeta);
        // front wheels (ahead of the COM) steer; rears don't
        w->mMaxSteerAngle   = (offX[i] > 0.01f) ? MAX_STEER : 0.0f;
        // front-biased braking, no ABS; handbrake acts on the rear axle only
        w->mMaxBrakeTorque  = (offX[i] > 0.01f) ? 3500.0f : 2500.0f;
        w->mMaxHandBrakeTorque = (offX[i] > 0.01f) ? 0.0f : 5000.0f;
        vs.mWheels.push_back(w);
    }

    WheeledVehicleControllerSettings* ctl = new WheeledVehicleControllerSettings();

    // engine curve (normalized: X = rpm fraction of maxRPM, Y = fraction of maxTorque)
    ctl->mEngine.mMaxTorque = std::max(1.0f, dt.maxTorque);
    ctl->mEngine.mMinRPM    = std::max(10.0f, dt.minRPM);
    ctl->mEngine.mMaxRPM    = std::max(dt.minRPM+100.0f, dt.maxRPM);
    ctl->mEngine.mInertia   = std::max(0.05f, dt.inertia);
    ctl->mEngine.mNormalizedTorque.Clear();
    if(dt.curveRPM.size()>=2 && dt.curveRPM.size()==dt.curveNm.size()){
        for(size_t i=0;i<dt.curveRPM.size();++i){
            float x = dt.curveRPM[i]/ctl->mEngine.mMaxRPM;
            float y = dt.curveNm[i] /ctl->mEngine.mMaxTorque;
            ctl->mEngine.mNormalizedTorque.AddPoint(std::max(0.0f,std::min(1.0f,x)),
                                                    std::max(0.0f,y));
        }
        ctl->mEngine.mNormalizedTorque.Sort();
    } else {
        ctl->mEngine.mNormalizedTorque.AddPoint(0.0f,0.8f);
        ctl->mEngine.mNormalizedTorque.AddPoint(0.7f,1.0f);
        ctl->mEngine.mNormalizedTorque.AddPoint(1.0f,0.8f);
    }

    // transmission: manual (the UI shifts); gears from the editable ratios
    ctl->mTransmission.mMode = ETransmissionMode::Manual;
    ctl->mTransmission.mGearRatios.clear();
    if(dt.gearRatios.empty()) ctl->mTransmission.mGearRatios.push_back(4.21f);
    else for(float g : dt.gearRatios) ctl->mTransmission.mGearRatios.push_back(g);
    ctl->mTransmission.mReverseGearRatios.clear();
    ctl->mTransmission.mReverseGearRatios.push_back(-std::fabs(dt.reverseRatio));
    // heavy single dry plate: takes a moment to bite, holds a lot of torque
    ctl->mTransmission.mClutchStrength = std::max(1.0f, dt.clutchStrength);
    ctl->mTransmission.mSwitchTime     = std::max(0.05f, dt.shiftTime);
    ctl->mTransmission.mClutchReleaseTime = std::max(0.05f, dt.shiftTime*0.8f);
    // diesel shift points (only consulted if the mode is switched to Auto)
    ctl->mTransmission.mShiftUpRPM   = 2800.0f;
    ctl->mTransmission.mShiftDownRPM = 1500.0f;

    // one differential per adjacent pair of driven wheels; the final drive is
    // folded into the differential ratio.
    std::vector<int> dv;
    for(size_t i=0;i<nW;++i) if(i<driven.size() && driven[i]) dv.push_back((int)i);
    if(dv.empty() && nW>0) dv.push_back((int)nW-1);   // always drive something
    ctl->mDifferentials.clear();
    for(size_t i=0;i<dv.size();i+=2){
        VehicleDifferentialSettings d;
        d.mLeftWheel  = dv[i];
        d.mRightWheel = (i+1<dv.size()) ? dv[i+1] : -1;
        d.mDifferentialRatio = std::max(0.1f, dt.finalDrive);
        // FLT_MAX = fully open (the Transit's stock rear axle)
        d.mLimitedSlipRatio  = dt.lsdRatio >= FLT_MAX*0.5f
                                 ? FLT_MAX : std::max(1.01f, dt.lsdRatio);
        ctl->mDifferentials.push_back(d);
    }
    // split engine torque evenly across the differentials
    float share = ctl->mDifferentials.empty()? 1.0f
                    : 1.0f/(float)ctl->mDifferentials.size();
    for(VehicleDifferentialSettings& d : ctl->mDifferentials) d.mEngineTorqueRatio=share;

    // remember what the stall model needs: which wheels the engine can be
    // dragged down by, and the reduction between them and the crankshaft
    p_->drivenIdx  = dv;
    p_->finalDrive = std::max(0.1f, dt.finalDrive);
    p_->stallRPM   = std::max(0.0f, dt.stallRPM);
    p_->stalled    = false;
    p_->stallT     = 0.0f;

    vs.mController = ctl;

    p_->vehicle = new VehicleConstraint(*b, vs);
    p_->tester  = new VehicleCollisionTesterCastCylinder(Layers::MOVING, 0.05f);
    p_->vehicle->SetVehicleCollisionTester(p_->tester);
    p_->sys.AddConstraint(p_->vehicle);
    p_->sys.AddStepListener(p_->vehicle);
}
bool World::hasVehicle() const { return p_->haveChassis; }
void World::setSusp(const Susp& s){ p_->susp=s; }

void World::resetVehicle(float x,float y,float z){
    if(!p_->haveChassis) return;
    BodyInterface& bi = p_->sys.GetBodyInterface();
    bi.SetPositionAndRotation(p_->chassis, RVec3(x,y,z), Quat::sIdentity(),
                              EActivation::Activate);
    bi.SetLinearVelocity (p_->chassis, Vec3::sZero());
    bi.SetAngularVelocity(p_->chassis, Vec3::sZero());
    p_->stalled = false;
    p_->stallT  = 0.0f;
    std::fill(p_->brakeStuck.begin(), p_->brakeStuck.end(), (char)0);
}

// ----------------------------- raycast car rig ------------------------------
// Jolt owns the chassis body, the suspension raycasts and the collision.  It
// does NOT own the drivetrain: no VehicleConstraint, no controller, no engine.
// The caller integrates its own engine/clutch/gearbox/diff and hands us the
// longitudinal tyre force each wheel is making.
void World::buildRig(const std::vector<float>& offX,
                     const std::vector<float>& offY,
                     const std::vector<float>& offZ,
                     float bodyLen,float bodyHei,float bodyWid,float mass,
                     float spawnX,float spawnY,float spawnZ,
                     float comX,float comY,float comZ){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    // tear down anything that was there before
    if(p_->vehicle!=nullptr){
        p_->sys.RemoveStepListener(p_->vehicle);
        p_->sys.RemoveConstraint(p_->vehicle);
        p_->vehicle=nullptr;
    }
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis);
                         p_->haveChassis=false; }

    size_t nW = offX.size();
    p_->ox=offX; p_->oy=offY; p_->oz=offZ;
    p_->spin.assign(nW, 0.0f);
    p_->brakeAnchor.assign(nW, Vec3::sZero());
    p_->brakeStuck.assign(nW, 0);
    p_->bodyLen=bodyLen; p_->bodyHei=bodyHei; p_->bodyWid=bodyWid; p_->mass=mass;
    p_->isCar = false; p_->isTank = false; p_->isRig = true;
    wout_.assign(nW, WheelOut{});

    // box chassis, wrapped so its centre of mass can be shifted (body space)
    BoxShapeSettings boxS(Vec3(bodyLen*0.5f, bodyHei*0.5f, bodyWid*0.5f));
    ShapeRefC boxShape = boxS.Create().Get();
    OffsetCenterOfMassShapeSettings comS(Vec3(comX,comY,comZ), boxShape);
    ShapeSettings::ShapeResult res = comS.Create();
    BodyCreationSettings bcs(res.Get(),
        RVec3(spawnX,spawnY,spawnZ), Quat::sIdentity(),
        EMotionType::Dynamic, Layers::MOVING);
    bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = mass;
    bcs.mLinearDamping  = 0.02f;
    bcs.mAngularDamping = 0.15f;
    bcs.mMotionQuality  = EMotionQuality::LinearCast;   // don't tunnel terrain
    Body* b = bi.CreateBody(bcs);
    p_->chassis = b->GetID();
    bi.AddBody(p_->chassis, EActivation::Activate);
    p_->haveChassis = true;
}

void World::stepRig(float dt, const std::vector<RigWheelIn>& in, float mu){
    if(p_->haveChassis){
        BodyInterface& bi = p_->sys.GetBodyInterface();
        RVec3 bp = bi.GetPosition(p_->chassis);
        Quat  bq = bi.GetRotation(p_->chassis);
        Vec3  up    = bq*Vec3(0,1,0);
        Vec3  fwd   = bq*Vec3(1,0,0);
        Vec3  right = bq*Vec3(0,0,1);

        const Susp& s = p_->susp;
        float rayLen = s.rest + s.radius;
        GroundOnlyLayerFilter& gf = p_->groundFilter;
        bool anyForce = false;

        for(size_t i=0;i<p_->ox.size();++i){
            RigWheelIn wi = (i<in.size()) ? in[i] : RigWheelIn{};
            // the wheel's own axes: heading swings with the steer angle
            float cs=std::cos(wi.steer), sn=std::sin(wi.steer);
            Vec3 wFwd   = fwd*cs + right*sn;
            Vec3 wRight = right*cs - fwd*sn;

            Vec3 local(p_->ox[i], p_->oy[i], p_->oz[i]);
            RVec3 attach = bp + bq*local;
            Vec3  down   = -up;

            RRayCast ray{ attach, down*rayLen };
            RayCastResult hit;
            bool grounded = p_->sys.GetNarrowPhaseQuery().CastRay(
                                ray, hit, BroadPhaseLayerFilter(), gf);

            WheelOut wo{};
            wo.steer = wi.steer;
            if(grounded){
                float d       = hit.mFraction*rayLen;
                float suspLen = std::max(0.0f, d - s.radius);
                float compression = std::max(0.0f, s.rest - suspLen);
                Vec3  contact = Vec3(attach) + down*d;
                Vec3  centre  = Vec3(attach) + down*(d - s.radius);

                // The load has to act along the GROUND normal, not along the
                // body's up axis.  With an offset centre of mass the van settles
                // slightly nose-down, and a body-aligned spring then tilts the
                // whole ~23 kN of support backwards with it -- a couple of
                // hundred newtons of thrust that crept the van backwards in
                // neutral, where free-rolling wheels leave nothing to resist it.
                Vec3 normal = up;
                {
                    BodyLockRead lock(p_->sys.GetBodyLockInterface(), hit.mBodyID);
                    if(lock.Succeeded())
                        normal = lock.GetBody().GetWorldSpaceSurfaceNormal(
                                     hit.mSubShapeID2, RVec3(contact));
                }
                if(normal.LengthSq() < 1.0e-6f) normal = up;
                else {
                    normal = normal.Normalized();
                    if(normal.Dot(up) < 0.0f) normal = -normal;
                }

                Vec3  vAtt  = bi.GetPointVelocity(p_->chassis, RVec3(attach));
                float upVel = vAtt.Dot(normal);

                // spring + damper, with a stiff bump stop past full travel
                float springTravel = std::min(compression, s.travel);
                float Fspring = s.stiffness*springTravel;
                float excess  = compression - s.travel;
                if(excess > 0.0f) Fspring += s.stiffness*8.0f*excess;   // bottoming out
                float Fdamp   = -s.damping*upVel;
                float Fz = std::max(0.0f, Fspring + Fdamp);
                bi.AddForce(p_->chassis, normal*Fz, RVec3(attach));

                // ground velocity at the contact, resolved in the wheel's axes
                Vec3  vCon = bi.GetPointVelocity(p_->chassis, RVec3(contact));
                float vx = vCon.Dot(wFwd);
                float vy = vCon.Dot(wRight);

                // friction circle: the drivetrain's longitudinal force gets
                // first claim, lateral grip takes whatever budget is left
                float cap = std::max(0.0f, mu*Fz);
                float Fx  = std::max(-cap, std::min(cap, wi.Fx));
                float latBudget = std::sqrt(std::max(0.0f, cap*cap - Fx*Fx));
                float Flat = -s.gripK*vy;
                Flat = std::max(-latBudget, std::min(latBudget, Flat));

                bi.AddForce(p_->chassis, wFwd*Fx + wRight*Flat, RVec3(contact));
                anyForce = true;

                wo.grounded=1; wo.Fz=Fz; wo.Fx=Fx; wo.vx=vx; wo.vy=vy;
                wo.compress = std::max(0.0f, std::min(1.0f,
                                compression/std::max(0.01f,s.travel)));
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            } else {
                // airborne: hang the wheel at full droop, no forces
                Vec3 centre = Vec3(attach) + down*s.rest;
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            }
            wout_[i]=wo;
        }
        if(anyForce) bi.ActivateBody(p_->chassis);
    }

    p_->sys.Update(dt, 2, p_->temp, p_->jobs);

    // refresh dynamic obstacle (crate) transforms for rendering
    BodyInterface& bi2 = p_->sys.GetBodyInterface();
    for(size_t i=0;i<p_->obsId.size();++i){
        if(!obs_[i].dynamic) continue;
        RVec3 p = bi2.GetPosition(p_->obsId[i]);
        Quat  q = bi2.GetRotation(p_->obsId[i]);
        obs_[i].px=p.GetX(); obs_[i].py=p.GetY(); obs_[i].pz=p.GetZ();
        obs_[i].qx=q.GetX(); obs_[i].qy=q.GetY(); obs_[i].qz=q.GetZ(); obs_[i].qw=q.GetW();
    }
}

// ----------------------------- tank -----------------------------------------
void World::buildTank(int roadWheelsPerSide, float x0, float x1,
                      float leftZ, float rightZ, float attachY,
                      float bodyLen,float bodyHei,float bodyWid,float mass,
                      float spawnX,float spawnY,float spawnZ,
                      float comX,float comY,float comZ){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis);
                         p_->haveChassis=false; }

    // lay out two rows of road-wheel rays: left row first, then right row
    int n = std::max(1, roadWheelsPerSide);
    p_->ox.clear(); p_->oy.clear(); p_->oz.clear(); p_->trackSide.clear();
    for(int side=0; side<2; ++side){
        float z = side==0 ? leftZ : rightZ;
        for(int k=0;k<n;++k){
            float t = (n>1) ? (float)k/(float)(n-1) : 0.5f;
            p_->ox.push_back(x0 + (x1-x0)*t);
            p_->oy.push_back(attachY);
            p_->oz.push_back(z);
            p_->trackSide.push_back((char)side);
        }
    }
    size_t nr = p_->ox.size();
    p_->brakeAnchor.assign(nr, Vec3::sZero());
    p_->brakeStuck.assign(nr, 0);
    wout_.assign(nr, WheelOut{});
    p_->bodyLen=bodyLen; p_->bodyHei=bodyHei; p_->bodyWid=bodyWid; p_->mass=mass;
    p_->isTank = true;
    p_->tout[0]=TrackOut{}; p_->tout[1]=TrackOut{};

    // ---- buoyancy sample grid ------------------------------------------
    // 3 (X) x 2 (Y) x 2 (Z) points through the hull ENVELOPE, which is bigger
    // than the collision box: a real tank displaces its sponsons and its two
    // track runs as well as the hull, and the tracks hang below the box, so the
    // envelope is also shifted down.  Points sit at slab centres, not on the
    // faces, so each one stands for an equal share of the volume around it.
    {
        const int NX=3, NY=2, NZ=2;
        float hx = bodyLen*0.58f;               // envelope half-extents
        float hy = bodyHei*0.80f;
        float hz = bodyWid*0.72f;
        float cy = -bodyHei*0.25f;              // envelope centre (tracks hang low)
        p_->buoyLocal.clear();
        p_->buoyLocal.reserve(NX*NY*NZ);
        auto slabCentre=[](int k,int n,float half){
            // k-th of n equal slabs spanning [-half,+half], at its centre
            return -half + (2.0f*half)*((float)k+0.5f)/(float)n;
        };
        for(int i=0;i<NX;i++) for(int j=0;j<NY;j++) for(int k=0;k<NZ;k++)
            p_->buoyLocal.push_back(Vec3(slabCentre(i,NX,hx),
                                         cy+slabCentre(j,NY,hy),
                                         slabCentre(k,NZ,hz)));
        // how tall a slice one point represents -- the depth over which its
        // submersion ramps 0 -> 1, which is what keeps the lift continuous
        p_->buoySlabH = std::max(0.05f, 2.0f*hy/(float)NY);
        bpts_.assign(p_->buoyLocal.size(), BuoyPoint{});
    }

    // box hull, wrapped so its centre of mass can be shifted (body space)
    BoxShapeSettings boxS(Vec3(bodyLen*0.5f, bodyHei*0.5f, bodyWid*0.5f));
    ShapeRefC boxShape = boxS.Create().Get();
    OffsetCenterOfMassShapeSettings comS(Vec3(comX,comY,comZ), boxShape);
    ShapeSettings::ShapeResult res = comS.Create();
    BodyCreationSettings bcs(res.Get(),
        RVec3(spawnX,spawnY,spawnZ), Quat::sIdentity(),
        EMotionType::Dynamic, Layers::MOVING);
    bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = mass;
    bcs.mLinearDamping  = 0.05f;
    bcs.mAngularDamping = 0.40f;   // extra yaw damping for skid-steer stability
    // continuous collision so the heavy hull can't tunnel through the terrain
    bcs.mMotionQuality  = EMotionQuality::LinearCast;
    Body* b = bi.CreateBody(bcs);
    p_->chassis = b->GetID();
    bi.AddBody(p_->chassis, EActivation::Activate);
    p_->haveChassis = true;
}

void World::setTrackSusp(const TrackSusp& s){ p_->tsusp=s; }
void World::setBuoy(const Buoy& b){ p_->buoy=b; }
const Buoy& World::buoy() const { return p_->buoy; }
const TrackOut& World::track(int side) const { return p_->tout[side&1]; }

void World::stepTank(float dt, float leftSurf, float rightSurf,
                     float brake, float mu){
    p_->tout[0]=TrackOut{}; p_->tout[1]=TrackOut{};

    if(p_->haveChassis){
        BodyInterface& bi = p_->sys.GetBodyInterface();
        RVec3 bp = bi.GetPosition(p_->chassis);
        Quat  bq = bi.GetRotation(p_->chassis);
        Vec3  up    = bq*Vec3(0,1,0);
        Vec3  fwd   = bq*Vec3(1,0,0);
        Vec3  right = bq*Vec3(0,0,1);

        const TrackSusp& s = p_->tsusp;
        float rayLen = s.rest + s.radius;
        GroundOnlyLayerFilter& gf = p_->groundFilter;

        for(size_t i=0;i<p_->ox.size();++i){
            int side = p_->trackSide[i];
            float surf = side==0 ? leftSurf : rightSurf;
            Vec3 local(p_->ox[i], p_->oy[i], p_->oz[i]);
            RVec3 attach = bp + bq*local;
            Vec3  down   = -up;

            RRayCast ray{ attach, down*rayLen };
            RayCastResult hit;
            bool grounded = p_->sys.GetNarrowPhaseQuery().CastRay(
                                ray, hit, BroadPhaseLayerFilter(), gf);

            WheelOut wo{};
            if(grounded){
                float d = hit.mFraction*rayLen;
                float suspLen = std::max(0.0f, d - s.radius);
                float compression = s.rest - suspLen;
                Vec3 contact = Vec3(attach) + down*d;
                Vec3 centre  = Vec3(attach) + down*(d - s.radius);

                Vec3 vAtt = bi.GetPointVelocity(p_->chassis, RVec3(attach));
                float upVel = vAtt.Dot(up);
                float Fspring = s.stiffness*compression;
                float Fdamp   = -s.damping*upVel;
                float Fz = std::max(0.0f, Fspring + Fdamp);
                bi.AddForce(p_->chassis, up*Fz, RVec3(attach));

                float cap = mu*Fz;

                // longitudinal thrust from track-vs-ground slip
                float vfwd  = vAtt.Dot(fwd);
                float vslip = surf - vfwd;
                float Ftr   = s.trackK*vslip;
                Ftr = std::max(-cap, std::min(cap, Ftr));
                bi.AddForce(p_->chassis, fwd*Ftr, RVec3(contact));

                // lateral grip opposes side slip (this is what makes it yaw)
                float vlat = vAtt.Dot(right);
                float Flat = -s.gripK*vlat;
                Flat = std::max(-cap, std::min(cap, Flat));
                bi.AddForce(p_->chassis, right*Flat, RVec3(contact));

                // brake: PD static-friction hold along the track-forward axis
                if(brake>0.001f){
                    if(!p_->brakeStuck[i]){ p_->brakeAnchor[i]=contact;
                                            p_->brakeStuck[i]=1; }
                    float xlong = (contact - p_->brakeAnchor[i]).Dot(fwd);
                    float Fb  = -(s.stiffness*xlong + s.damping*vfwd)*brake;
                    if(std::fabs(Fb) > cap){
                        Fb = Fb>0 ? cap : -cap;
                        p_->brakeAnchor[i]=contact;
                    }
                    bi.AddForce(p_->chassis, fwd*Fb, RVec3(contact));
                } else {
                    p_->brakeStuck[i]=0;
                }

                TrackOut& to = p_->tout[side];
                to.force += Ftr; to.load += Fz;
                to.groundedRays++; to.contactSpeed += vfwd;

                wo.grounded=1; wo.Fz=Fz;
                wo.compress = std::max(0.0f,std::min(1.0f, compression/std::max(0.01f,s.travel)));
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            } else {
                p_->brakeStuck[i]=0;
                Vec3 centre = Vec3(attach) + down*s.rest;
                wo.grounded=0; wo.Fz=0; wo.compress=0;
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            }
            if(i<wout_.size()) wout_[i]=wo;
        }

        for(int t=0;t<2;t++)
            if(p_->tout[t].groundedRays>0)
                p_->tout[t].contactSpeed /= (float)p_->tout[t].groundedRays;

        // ---- water: buoyancy, drag and track paddling ---------------------
        // Runs AFTER the suspension so the two just add up.  Nothing here is
        // conditional on "being in the lake": every point simply asks how deep
        // it is, which is exactly why driving down a ramp into the water is a
        // smooth handover -- the front points get lift first and the springs
        // unload themselves as the hull starts to swim.
        const Buoy& bo = p_->buoy;
        float subAvg = 0.0f;
        if(bo.enabled && !p_->buoyLocal.empty()){
            const size_t N   = p_->buoyLocal.size();
            const float  rhoG = 9810.0f;             // rho*g  [N/m^3]
            const float  Vi   = bo.volume/(float)N;  // volume one point carries
            const float  half = 0.5f*p_->buoySlabH;
            const float  kLin = bo.linDrag  /(float)N;
            const float  kHev = bo.heaveDamp/(float)N;

            for(size_t i=0;i<N;++i){
                RVec3 pw  = bp + bq*p_->buoyLocal[i];
                Vec3  pwv = Vec3(pw);
                // fraction of this point's slab that is under the surface
                float sub = (bo.level - (pwv.GetY()-half)) / p_->buoySlabH;
                sub = std::max(0.0f, std::min(1.0f, sub));

                BuoyPoint& op = bpts_[i];
                op.x=pwv.GetX(); op.y=pwv.GetY(); op.z=pwv.GetZ(); op.sub=sub;
                op.fx=op.fy=op.fz=0; op.dx=op.dy=op.dz=0;
                subAvg += sub;
                if(sub<=0.0f) continue;

                Vec3 Fb(0.0f, rhoG*Vi*sub, 0.0f);
                bi.AddForce(p_->chassis, Fb, pw);
                op.fy = Fb.GetY();

                // drag: horizontal resistance sets the swimming top speed,
                // the (much stiffer) vertical term stops the hull pogoing on
                // the surface -- buoyancy alone is an undamped spring.
                Vec3 v  = bi.GetPointVelocity(p_->chassis, pw);
                Vec3 Fd(-kLin*sub*v.GetX(),
                        -kHev*sub*v.GetY(),
                        -kLin*sub*v.GetZ());
                bi.AddForce(p_->chassis, Fd, pw);
                op.dx=Fd.GetX(); op.dy=Fd.GetY(); op.dz=Fd.GetZ();
            }
            subAvg /= (float)N;

            if(subAvg>0.0f){
                // angular water resistance (yaw/roll/pitch alike)
                bi.AddTorque(p_->chassis,
                             bi.GetAngularVelocity(p_->chassis)*(-bo.angDrag*subAvg));

                // track paddling: submerged tracks throw water backwards, which
                // is how an amphibious tank swims.  Applying it per side at that
                // track's own Z keeps skid-steer working afloat.
                for(int side=0; side<2; ++side){
                    float surf = side==0 ? leftSurf : rightSurf;
                    if(std::fabs(surf)<1e-3f) continue;
                    surf = std::max(-bo.paddleCap, std::min(bo.paddleCap, surf));
                    float zc=0.0f; int n=0;
                    for(size_t i=0;i<p_->oz.size();++i)
                        if(p_->trackSide[i]==side){ zc+=p_->oz[i]; n++; }
                    if(n) zc/=(float)n;
                    RVec3 at = bp + bq*Vec3(-p_->bodyLen*0.25f, p_->oy.empty()?0.0f:p_->oy[0], zc);
                    bi.AddForce(p_->chassis, fwd*(bo.paddle*surf*subAvg*0.5f), at);
                }
            }
        } else {
            for(size_t i=0;i<bpts_.size();++i) bpts_[i]=BuoyPoint{};
        }
    }

    p_->sys.Update(dt, 4, p_->temp, p_->jobs);   // 4 substeps for the heavy body

    BodyInterface& bi2 = p_->sys.GetBodyInterface();
    for(size_t i=0;i<p_->obsId.size();++i){
        if(!obs_[i].dynamic) continue;
        RVec3 p = bi2.GetPosition(p_->obsId[i]);
        Quat  q = bi2.GetRotation(p_->obsId[i]);
        obs_[i].px=p.GetX(); obs_[i].py=p.GetY(); obs_[i].pz=p.GetZ();
        obs_[i].qx=q.GetX(); obs_[i].qy=q.GetY(); obs_[i].qz=q.GetZ(); obs_[i].qw=q.GetW();
    }
}

// ----------------------------- obstacles ------------------------------------
void World::addCrate(float x,float y,float z,float half,float mass){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    BoxShapeSettings bs(Vec3(half,half,half));
    BodyCreationSettings bcs(bs.Create().Get(), RVec3(x,y,z), Quat::sIdentity(),
                             EMotionType::Dynamic, Layers::MOVING);
    bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = mass;
    Body* b = bi.CreateBody(bcs);
    bi.AddBody(b->GetID(), EActivation::Activate);
    p_->obsId.push_back(b->GetID());
    ObstacleOut o; o.kind=0; o.px=x;o.py=y;o.pz=z; o.qw=1;
    o.sx=o.sy=o.sz=half; o.dynamic=1;
    obs_.push_back(o); p_->obsInit.push_back(o);
}

void World::addRamp(float x,float y,float z,float yawRad,
                    float L,float h,float w){
    float hL=L*0.5f, hw=w*0.5f;
    Array<Vec3> pts;                         // centred triangular prism
    pts.push_back(Vec3(-hL,0,-hw)); pts.push_back(Vec3(-hL,0, hw));
    pts.push_back(Vec3( hL,0,-hw)); pts.push_back(Vec3( hL,0, hw));
    pts.push_back(Vec3( hL,h,-hw)); pts.push_back(Vec3( hL,h, hw));
    ConvexHullShapeSettings hs(pts, 0.0f);
    BodyInterface& bi = p_->sys.GetBodyInterface();
    Quat q = Quat::sRotation(Vec3(0,1,0), yawRad);
    Body* b = bi.CreateBody(BodyCreationSettings(hs.Create().Get(),
                RVec3(x,y,z), q, EMotionType::Static, Layers::NON_MOVING));
    bi.AddBody(b->GetID(), EActivation::DontActivate);
    p_->obsId.push_back(b->GetID());
    ObstacleOut o; o.kind=1; o.px=x;o.py=y;o.pz=z;
    o.qx=q.GetX();o.qy=q.GetY();o.qz=q.GetZ();o.qw=q.GetW();
    o.sx=L;o.sy=h;o.sz=w; o.dynamic=0;
    obs_.push_back(o); p_->obsInit.push_back(o);
}

void World::resetObstacles(){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    for(size_t i=0;i<p_->obsId.size();++i){
        if(!obs_[i].dynamic) continue;
        const ObstacleOut& s=p_->obsInit[i];
        bi.SetPositionAndRotation(p_->obsId[i], RVec3(s.px,s.py,s.pz),
            Quat(s.qx,s.qy,s.qz,s.qw), EActivation::Activate);
        bi.SetLinearVelocity (p_->obsId[i], Vec3::sZero());
        bi.SetAngularVelocity(p_->obsId[i], Vec3::sZero());
        obs_[i]=s;
    }
}

void World::step(float dt, float throttle, float brake, float steer,
                 int gear, float clutch, float mu){
    if(p_->haveChassis && p_->vehicle!=nullptr){
        BodyInterface& bi = p_->sys.GetBodyInterface();
        WheeledVehicleController* c =
            static_cast<WheeledVehicleController*>(p_->vehicle->GetController());

        // surface grip: Jolt combines the wheel-friction curve with the ground
        // body's friction, so we steer grip through the ground contact.
        if(!p_->ground.IsInvalid()) bi.SetFriction(p_->ground, std::max(0.02f,mu));

        float fwd    = std::max(0.0f, std::min(1.0f, throttle));
        float rt     = std::max(-1.0f, std::min(1.0f, steer));
        float br     = std::max(0.0f, std::min(1.0f, brake));
        float cl     = std::max(0.0f, std::min(1.0f, clutch));

        // ---- stall model ---------------------------------------------------
        // Jolt's engine is clamped to mMinRPM, so on its own it would sit at
        // idle forever and keep shoving torque through an engaged clutch --
        // the wheels fight the brakes instead of the engine dying.  With the
        // clutch out the driveline dictates crank speed, so work out what rpm
        // the wheels are asking for and kill the engine if the brakes drag it
        // below the stall threshold.
        bool inGear = (gear != 0);
        if(!inGear || cl < 0.15f){          // clutch in or neutral -> it restarts
            p_->stalled = false;
            p_->stallT  = 0.0f;
        } else if(!p_->stalled){
            float ratio = std::fabs(c->GetTransmission().GetCurrentRatio())
                          * p_->finalDrive;
            float wOmega = 0.0f;  int n = 0;
            for(int wi : p_->drivenIdx){
                if(wi>=0 && wi<(int)p_->vehicle->GetWheels().size()){
                    wOmega += std::fabs(p_->vehicle->GetWheel(wi)->GetAngularVelocity());
                    n++;
                }
            }
            if(n>0) wOmega /= (float)n;
            float rpmDemand = wOmega*ratio*60.0f/(2.0f*JPH_PI);
            // only a nearly-engaged clutch can drag the crank down; slipping it
            // (a normal launch) leaves the engine free to rev.
            bool dragged = cl > 0.85f && rpmDemand < p_->stallRPM && br > 0.15f;
            p_->stallT = dragged ? p_->stallT + dt : 0.0f;
            if(p_->stallT > 0.15f) p_->stalled = true;
        }

        if(p_->stalled){
            // dead engine: no combustion torque, and the locked driveline holds
            // the vehicle exactly where it stopped.
            fwd = 0.0f;
            br  = 1.0f;
            cl  = 0.0f;      // nothing gets through to the wheels
        }

        c->SetDriverInput(fwd, rt, br, 0.0f);
        c->GetTransmission().Set(gear, cl);

        // keep the chassis awake whenever the driver is asking for something
        if(fwd>0.01f || br>0.01f || std::fabs(rt)>0.01f)
            bi.ActivateBody(p_->chassis);
    }

    p_->sys.Update(dt, 2, p_->temp, p_->jobs);

    // read the Jolt wheels back into WheelOut for rendering / telemetry
    if(p_->haveChassis && p_->vehicle!=nullptr){
        const Susp& s = p_->susp;
        float travel  = std::max(0.01f, s.travel);
        float invDt   = 1.0f/std::max(1e-4f,dt);
        for(uint i=0;i<(uint)p_->ox.size() && i<wout_.size();++i){
            const WheelWV* w = static_cast<const WheelWV*>(p_->vehicle->GetWheel(i));
            RMat44 wt = p_->vehicle->GetWheelWorldTransform(i, Vec3(0,0,1), Vec3(0,1,0));
            RVec3 cw = wt.GetTranslation();
            WheelOut wo{};
            wo.x=(float)cw.GetX(); wo.y=(float)cw.GetY(); wo.z=(float)cw.GetZ();
            wo.grounded = w->HasContact() ? 1 : 0;
            wo.Fz       = w->GetSuspensionLambda()  * invDt;   // impulse -> force
            wo.Fx       = w->GetLongitudinalLambda()* invDt;
            wo.compress = std::max(0.0f, std::min(1.0f,
                            (s.rest - w->GetSuspensionLength())/travel));
            wo.steer    = w->GetSteerAngle();
            wo.spin     = w->GetRotationAngle();
            wo.slip     = w->mLongitudinalSlip;
            wo.omega    = w->GetAngularVelocity();
            wout_[i]=wo;
        }
    }

    // refresh dynamic obstacle (crate) transforms for rendering
    BodyInterface& bi2 = p_->sys.GetBodyInterface();
    for(size_t i=0;i<p_->obsId.size();++i){
        if(!obs_[i].dynamic) continue;
        RVec3 p = bi2.GetPosition(p_->obsId[i]);
        Quat  q = bi2.GetRotation(p_->obsId[i]);
        obs_[i].px=p.GetX(); obs_[i].py=p.GetY(); obs_[i].pz=p.GetZ();
        obs_[i].qx=q.GetX(); obs_[i].qy=q.GetY(); obs_[i].qz=q.GetZ(); obs_[i].qw=q.GetW();
    }
}

// ----------------------------- read-back ------------------------------------
void World::bodyPosition(float o[3]) const {
    if(!p_->haveChassis){ o[0]=o[1]=o[2]=0; return; }
    RVec3 p = p_->sys.GetBodyInterface().GetPosition(p_->chassis);  // shape origin
    o[0]=p.GetX(); o[1]=p.GetY(); o[2]=p.GetZ();
}
void World::bodyQuat(float o[4]) const {
    if(!p_->haveChassis){ o[0]=o[1]=o[2]=0; o[3]=1; return; }
    Quat q = p_->sys.GetBodyInterface().GetRotation(p_->chassis);
    o[0]=q.GetX(); o[1]=q.GetY(); o[2]=q.GetZ(); o[3]=q.GetW();
}
void World::bodyDims(float o[3]) const { o[0]=p_->bodyLen; o[1]=p_->bodyHei; o[2]=p_->bodyWid; }
float World::forwardSpeed() const {
    if(!p_->haveChassis) return 0.0f;
    BodyInterface& bi = p_->sys.GetBodyInterface();
    Vec3 v = bi.GetLinearVelocity(p_->chassis);
    Vec3 fwd = bi.GetRotation(p_->chassis)*Vec3(1,0,0);
    return v.Dot(fwd);
}
float World::engineRPM() const {
    if(p_->vehicle==nullptr) return 0.0f;
    if(p_->stalled) return 0.0f;                  // dead engine, no crank speed
    const WheeledVehicleController* c =
        static_cast<const WheeledVehicleController*>(p_->vehicle->GetController());
    return c->GetEngine().GetCurrentRPM();
}
bool World::engineStalled() const { return p_->stalled; }
int World::currentGear() const {
    if(p_->vehicle==nullptr) return 0;
    const WheeledVehicleController* c =
        static_cast<const WheeledVehicleController*>(p_->vehicle->GetController());
    return c->GetTransmission().GetCurrentGear();
}

} // namespace phys
