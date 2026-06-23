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
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>

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
Susp suspPreset(int which){
    Susp s;
    switch(which){
        case SUSP_COMFORT: s={0.46f,0.26f,22000.f,4200.f,0.42f,12000.f}; break;
        case SUSP_OFFROAD: s={0.55f,0.40f,26000.f,4600.f,0.46f,13000.f}; break;
        case SUSP_SPORT:
        default:           s={0.36f,0.16f,40000.f,5200.f,0.40f,20000.f}; break;
    }
    return s;
}
const char* suspPresetName(int which){
    switch(which){ case SUSP_COMFORT: return "COMFORT";
                   case SUSP_OFFROAD: return "OFFROAD";
                   default:           return "SPORT"; }
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

    // terrain
    int   hmN    = 0;
    float hmCell = 2.0f;
    std::vector<float> heights;

    // wheels (body-space attach offsets)
    std::vector<float> ox, oy, oz;
    float bodyLen=4, bodyHei=0.8f, bodyWid=1.8f, mass=1300;
    Susp susp;
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
                         float bodyLen,float bodyHei,float bodyWid,float mass,
                         float spawnX,float spawnY,float spawnZ){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis);
                         p_->haveChassis=false; }

    p_->ox=offX; p_->oy=offY; p_->oz=offZ;
    p_->bodyLen=bodyLen; p_->bodyHei=bodyHei; p_->bodyWid=bodyWid; p_->mass=mass;
    wout_.assign(offX.size(), WheelOut{});

    BoxShapeSettings boxS(Vec3(bodyLen*0.5f, bodyHei*0.5f, bodyWid*0.5f));
    ShapeSettings::ShapeResult res = boxS.Create();
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
}

void World::step(float dt, const std::vector<float>& driveN,
                 float brake, float steerRad, float mu){
    if(p_->haveChassis){
        BodyInterface& bi = p_->sys.GetBodyInterface();
        RVec3 bp = bi.GetCenterOfMassPosition(p_->chassis);
        Quat  bq = bi.GetRotation(p_->chassis);
        Vec3  up    = bq*Vec3(0,1,0);
        Vec3  fwd   = bq*Vec3(1,0,0);
        Vec3  right = bq*Vec3(0,0,1);

        const Susp& s = p_->susp;
        float rayLen = s.rest + s.radius;
        GroundOnlyLayerFilter& gf = p_->groundFilter;

        for(size_t i=0;i<p_->ox.size();++i){
            Vec3 local(p_->ox[i], p_->oy[i], p_->oz[i]);
            RVec3 attach = bp + bq*local;
            Vec3  down   = -up;

            RRayCast ray{ attach, down*rayLen };
            RayCastResult hit;
            bool grounded = p_->sys.GetNarrowPhaseQuery().CastRay(
                                ray, hit, BroadPhaseLayerFilter(), gf);

            WheelOut wo{};
            if(grounded){
                float d = hit.mFraction*rayLen;          // attach -> ground
                float suspLen = std::max(0.0f, d - s.radius);
                float compression = s.rest - suspLen;     // + = compressed
                // wheel centre sits one radius above the ground hit
                Vec3 contact = Vec3(attach) + down*d;
                Vec3 centre  = Vec3(attach) + down*(d - s.radius);

                Vec3 vAtt = bi.GetPointVelocity(p_->chassis, RVec3(attach));
                float upVel = vAtt.Dot(up);
                float Fspring = s.stiffness*compression;
                float Fdamp   = -s.damping*upVel;
                float Fz = std::max(0.0f, Fspring + Fdamp);

                bi.AddForce(p_->chassis, up*Fz, RVec3(attach));

                // steer the front wheels (offX>0) by rotating the tyre basis
                Vec3 wf=fwd, wr=right;
                if(p_->ox[i]>0.01f && std::fabs(steerRad)>1e-4f){
                    float c=std::cos(steerRad), sn=std::sin(steerRad);
                    wf = fwd*c + right*sn;
                    wr = right*c - fwd*sn;
                }

                // longitudinal tyre force from the drivetrain
                float drive = i<driveN.size()? driveN[i] : 0.0f;
                bi.AddForce(p_->chassis, wf*drive, RVec3(contact));

                // lateral grip opposes sideways slip, capped by mu*Fz
                float vlat = vAtt.Dot(wr);
                float Flat = -s.gripK*vlat;
                float cap  = mu*Fz;
                Flat = std::max(-cap, std::min(cap, Flat));
                bi.AddForce(p_->chassis, wr*Flat, RVec3(contact));

                // brake: damp forward motion at the contact
                if(brake>0.001f){
                    float vfwd = vAtt.Dot(wf);
                    float Fb = -brake*2200.0f*std::tanh(vfwd*2.0f);
                    bi.AddForce(p_->chassis, wf*Fb, RVec3(contact));
                }

                wo.grounded=1; wo.Fz=Fz;
                wo.compress = std::max(0.0f,std::min(1.0f, compression/std::max(0.01f,s.travel)));
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            } else {
                Vec3 centre = Vec3(attach) + down*s.rest;
                wo.grounded=0; wo.Fz=0; wo.compress=0;
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            }
            if(i<wout_.size()) wout_[i]=wo;
        }
    }

    p_->sys.Update(dt, 2, p_->temp, p_->jobs);
}

// ----------------------------- read-back ------------------------------------
void World::bodyPosition(float o[3]) const {
    if(!p_->haveChassis){ o[0]=o[1]=o[2]=0; return; }
    RVec3 p = p_->sys.GetBodyInterface().GetCenterOfMassPosition(p_->chassis);
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

} // namespace phys
