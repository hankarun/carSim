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

    // tank track state
    std::vector<char> trackSide;   // per ray: 0 = left track, 1 = right track
    TrackSusp         tsusp;
    TrackOut          tout[2];     // per-track aggregate readback (0=L,1=R)
    bool              isTank = false;
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
                         float bodyLen,float bodyHei,float bodyWid,float mass,
                         float spawnX,float spawnY,float spawnZ,
                         float comX,float comY,float comZ){
    BodyInterface& bi = p_->sys.GetBodyInterface();
    if(p_->haveChassis){ bi.RemoveBody(p_->chassis); bi.DestroyBody(p_->chassis);
                         p_->haveChassis=false; }

    p_->ox=offX; p_->oy=offY; p_->oz=offZ;
    p_->brakeAnchor.assign(offX.size(), Vec3::sZero());
    p_->brakeStuck.assign(offX.size(), 0);
    p_->bodyLen=bodyLen; p_->bodyHei=bodyHei; p_->bodyWid=bodyWid; p_->mass=mass;
    wout_.assign(offX.size(), WheelOut{});

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
    std::fill(p_->brakeStuck.begin(), p_->brakeStuck.end(), (char)0);
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

void World::step(float dt, const std::vector<float>& driveN,
                 float brake, float steerRad, float mu){
    if(p_->haveChassis){
        BodyInterface& bi = p_->sys.GetBodyInterface();
        RVec3 bp = bi.GetPosition(p_->chassis);     // shape origin (box centre)
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

                // brake: a static-friction hold along the wheel-forward axis.
                // We anchor the contact point and pull it back with a PD force
                // (spring+damper) so a stopped car HOLDS on a slope instead of
                // creeping down it.  The force is capped by the available grip
                // (mu*Fz); once exceeded the anchor slides and the tyre skids.
                if(brake>0.001f){
                    float vfwd = vAtt.Dot(wf);
                    if(!p_->brakeStuck[i]){ p_->brakeAnchor[i]=contact;
                                            p_->brakeStuck[i]=1; }
                    float xlong = (contact - p_->brakeAnchor[i]).Dot(wf);
                    float Fb  = -(s.stiffness*xlong + s.damping*vfwd)*brake;
                    float cap = mu*Fz;
                    if(std::fabs(Fb) > cap){              // grip lost -> skid
                        Fb = Fb>0 ? cap : -cap;
                        p_->brakeAnchor[i]=contact;       // let the anchor slide
                    }
                    bi.AddForce(p_->chassis, wf*Fb, RVec3(contact));
                } else {
                    p_->brakeStuck[i]=0;
                }

                wo.grounded=1; wo.Fz=Fz;
                wo.compress = std::max(0.0f,std::min(1.0f, compression/std::max(0.01f,s.travel)));
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            } else {
                p_->brakeStuck[i]=0;                  // airborne -> no static hold
                Vec3 centre = Vec3(attach) + down*s.rest;
                wo.grounded=0; wo.Fz=0; wo.compress=0;
                wo.x=centre.GetX(); wo.y=centre.GetY(); wo.z=centre.GetZ();
            }
            if(i<wout_.size()) wout_[i]=wo;
        }
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

} // namespace phys
