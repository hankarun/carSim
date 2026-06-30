// =============================================================================
//  ship.cpp -- hull geometry, custom 6-DOF integrator, triangle-center buoyancy.
//  See ship.h.  Buoyancy uses the hydrostatic pressure form: each submerged
//  facet is pushed along -outwardNormal by rho*g*depth*area, so the summed
//  vertical force equals rho*g*(displaced volume) and pitch/roll/heave emerge
//  naturally from the distribution.
// =============================================================================
#include "ship.h"
#include "raymath.h"
#include <cmath>

// ---- hull dimensions (body space; +X fwd, +Y up, +Z right) -----------------
static const float HULL_AFT  = 6.0f;   // stern at x = -AFT
static const float HULL_FWD  = 6.0f;   // bow   at x = +FWD
static const float HULL_BEAM = 2.2f;   // half beam
static const float HULL_KEEL =-1.4f;   // bottom y
static const float HULL_DECK = 0.9f;   // top y

// half-beam as a function of station x (tapers to a point at the bow)
static float halfBeam(float x){
    const float taper=3.0f;
    if(x<=taper) return HULL_BEAM;
    float f=(x-taper)/(HULL_FWD-taper);
    return HULL_BEAM*(1.0f-f) + 0.12f*f;
}

void shipInit(Ship& s){
    s.hull.clear();
    auto T=[&](Vector3 a,Vector3 b,Vector3 c){ s.hull.push_back({a,b,c}); };
    auto quad=[&](Vector3 p0,Vector3 p1,Vector3 p2,Vector3 p3){ T(p0,p1,p2); T(p0,p2,p3); };
    const float k=HULL_KEEL, d=HULL_DECK;
    // Tessellate finely: stations along length (pitch), vertical side segments
    // (so the waterline crosses the hull smoothly during heave) and beam
    // segments on the flat bottom/deck (smoother roll + more buoyancy samples).
    const int ns=12, nv=5, nb=3;
    auto X=[&](int i){ return -HULL_AFT + (HULL_FWD+HULL_AFT)*(float)i/ns; };

    for(int i=0;i<ns;++i){
        float x0=X(i), x1=X(i+1);
        float w0=halfBeam(x0), w1=halfBeam(x1);
        for(int j=0;j<nb;++j){
            float f0=(float)j/nb, f1=(float)(j+1)/nb;
            // bottom (y=k)
            quad({x0,k,-w0+2*w0*f0},{x0,k,-w0+2*w0*f1},
                 {x1,k,-w1+2*w1*f1},{x1,k,-w1+2*w1*f0});
            // deck (y=d)
            quad({x1,d,-w1+2*w1*f0},{x1,d,-w1+2*w1*f1},
                 {x0,d,-w0+2*w0*f1},{x0,d,-w0+2*w0*f0});
        }
        for(int v=0;v<nv;++v){
            float y0=k+(d-k)*v/nv, y1=k+(d-k)*(v+1)/nv;
            // left side (z=-w)
            quad({x0,y0,-w0},{x1,y0,-w1},{x1,y1,-w1},{x0,y1,-w0});
            // right side (z=+w)
            quad({x0,y0,w0},{x0,y1,w0},{x1,y1,w1},{x1,y0,w1});
        }
    }
    // transom (stern cap), subdivided vertically
    {
        float w=halfBeam(-HULL_AFT);
        for(int v=0;v<nv;++v){
            float y0=k+(d-k)*v/nv, y1=k+(d-k)*(v+1)/nv;
            quad({-HULL_AFT,y0,-w},{-HULL_AFT,y1,-w},{-HULL_AFT,y1,w},{-HULL_AFT,y0,w});
        }
    }
    // bow cap (small, near the point)
    {
        float w=halfBeam(HULL_FWD);
        quad({HULL_FWD,k,-w},{HULL_FWD,k,w},{HULL_FWD,d,w},{HULL_FWD,d,-w});
    }

    s.hullLen   = HULL_FWD+HULL_AFT;
    s.hullBeam  = 2.0f*HULL_BEAM;
    s.hullDepth = HULL_DECK-HULL_KEEL;
    s.insideRef = {0.0f,(HULL_KEEL+HULL_DECK)*0.5f,0.0f};
    // propeller on a shaft below the keel + rudder behind it, so both stay
    // submerged at the floating waterline
    s.propPos   = {-HULL_AFT-0.2f, HULL_KEEL-0.4f, 0.0f};
    s.rudderPos = {-HULL_AFT-0.6f, HULL_KEEL-0.2f, 0.0f};

    // The tapered bow has tiny facets, so the centre of buoyancy (the vertical
    // buoyancy is carried by the downward-facing bottom facets) sits AFT of the
    // geometric origin -> a bow-down trim couple if the COM stays at x=0.
    // Re-centre the body origin on that buoyancy centroid so the ship floats
    // level.  Weight each facet by area * (downward component of its normal).
    {
        double wsum=0.0, wx=0.0, wz=0.0;
        for(const Tri& t : s.hull){
            Vector3 ctr=Vector3Scale(Vector3Add(Vector3Add(t.a,t.b),t.c),1.0f/3.0f);
            Vector3 nn=Vector3Normalize(Vector3CrossProduct(
                        Vector3Subtract(t.b,t.a),Vector3Subtract(t.c,t.a)));
            if(Vector3DotProduct(nn,Vector3Subtract(ctr,s.insideRef))<0) nn=Vector3Negate(nn);
            float area=0.5f*Vector3Length(Vector3CrossProduct(
                        Vector3Subtract(t.b,t.a),Vector3Subtract(t.c,t.a)));
            double w=area*(double)std::max(0.0f,-nn.y);   // downward-facing share
            wsum+=w; wx+=w*ctr.x; wz+=w*ctr.z;
        }
        if(wsum>1e-6){
            Vector3 off={(float)(wx/wsum),0.0f,(float)(wz/wsum)};
            for(Tri& t : s.hull){
                t.a=Vector3Subtract(t.a,off);
                t.b=Vector3Subtract(t.b,off);
                t.c=Vector3Subtract(t.c,off);
            }
            s.propPos  =Vector3Subtract(s.propPos,off);
            s.rudderPos=Vector3Subtract(s.rudderPos,off);
            s.insideRef=Vector3Subtract(s.insideRef,off);
        }
    }

    // box-approx inertia tensor (diagonal)
    float L=s.hullLen, B=s.hullBeam, H=s.hullDepth, m=s.mass;
    float Ixx=m/12.0f*(B*B+H*H);
    float Iyy=m/12.0f*(L*L+B*B);
    float Izz=m/12.0f*(L*L+H*H);
    s.invInertiaDiag={1.0f/Ixx,1.0f/Iyy,1.0f/Izz};

    shipReset(s);
}

void shipBuildMesh(Ship& s){
    int tn=(int)s.hull.size();
    Mesh m{};
    m.vertexCount   = tn*3;
    m.triangleCount = tn;
    m.vertices=(float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.normals =(float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.colors  =(unsigned char*)MemAlloc(m.vertexCount*4*sizeof(unsigned char));
    for(int i=0;i<tn;++i){
        const Tri& t=s.hull[i];
        Vector3 n=Vector3Normalize(Vector3CrossProduct(
                    Vector3Subtract(t.b,t.a),Vector3Subtract(t.c,t.a)));
        Vector3 ctr=Vector3Scale(Vector3Add(Vector3Add(t.a,t.b),t.c),1.0f/3.0f);
        if(Vector3DotProduct(n,Vector3Subtract(ctr,s.insideRef))<0) n=Vector3Negate(n);
        Vector3 vs[3]={t.a,t.b,t.c};
        // hull grey, deck a touch lighter for readability
        unsigned char cr=170,cg=174,cb=184;
        if(ctr.y>0.4f){ cr=120;cg=128;cb=140; }
        for(int kk=0;kk<3;++kk){
            int v=i*3+kk;
            m.vertices[v*3+0]=vs[kk].x; m.vertices[v*3+1]=vs[kk].y; m.vertices[v*3+2]=vs[kk].z;
            m.normals[v*3+0]=n.x; m.normals[v*3+1]=n.y; m.normals[v*3+2]=n.z;
            m.colors[v*4+0]=cr; m.colors[v*4+1]=cg; m.colors[v*4+2]=cb; m.colors[v*4+3]=255;
        }
    }
    UploadMesh(&m,false);
    s.model=LoadModelFromMesh(m);
    s.built=true;
}

void shipReset(Ship& s){
    s.pos={0,0.4f,0};                         // ~floating equilibrium
    s.orient={0,0,0,1};
    s.vel={0,0,0};
    s.angVel={0,0,0};
    s.throttle=0; s.rudder=0; s.rudderAngle=0;
    s.propOmega=0; s.propAngle=0;
}

float shipForwardSpeed(const Ship& s){
    Vector3 fwd=Vector3RotateByQuaternion({1,0,0},s.orient);
    return Vector3DotProduct(s.vel,fwd);
}

void shipStep(Ship& s, const Ocean& o, float dt){
    // controls: smooth rudder toward target (±~0.6 rad), spin propeller
    s.rudderAngle += (s.rudder*0.6f - s.rudderAngle)*0.12f;
    s.propOmega    = s.maxPropOmega*s.throttle;
    s.propAngle   += s.propOmega*dt;

    Vector3 force ={0.0f,-s.mass*9.81f,0.0f};
    Vector3 torque={0.0f,0.0f,0.0f};
    auto addF=[&](Vector3 F,Vector3 worldPt){
        force=Vector3Add(force,F);
        Vector3 r=Vector3Subtract(worldPt,s.pos);
        torque=Vector3Add(torque,Vector3CrossProduct(r,F));
    };

    if(s.recordDebug) s.dbg.clear();

    Vector3 refInW=Vector3Add(s.pos,Vector3RotateByQuaternion(s.insideRef,s.orient));
    Vector3 fwd =Vector3RotateByQuaternion({1,0,0},s.orient);
    Vector3 side=Vector3RotateByQuaternion({0,0,1},s.orient);

    // ---- buoyancy + facet drag from each hull triangle center ----
    float totArea=0.0f, subArea=0.0f;          // for hull submersion fraction
    for(const Tri& tb : s.hull){
        Vector3 A=Vector3Add(s.pos,Vector3RotateByQuaternion(tb.a,s.orient));
        Vector3 B=Vector3Add(s.pos,Vector3RotateByQuaternion(tb.b,s.orient));
        Vector3 C=Vector3Add(s.pos,Vector3RotateByQuaternion(tb.c,s.orient));
        Vector3 center=Vector3Scale(Vector3Add(Vector3Add(A,B),C),1.0f/3.0f);
        Vector3 cross=Vector3CrossProduct(Vector3Subtract(B,A),Vector3Subtract(C,A));
        float area=0.5f*Vector3Length(cross);
        totArea+=area;
        float waterY=oceanSampleHeight(o,center.x,center.z);
        float depth=waterY-center.y;
        if(depth<=0.0f){                       // facet above the surface
            if(s.recordDebug) s.dbg.push_back({center,{0,0,0},5});  // dry sample
            continue;
        }
        subArea+=area;
        // outward normal (hydrostatic pressure pushes opposite to it)
        Vector3 n=Vector3Normalize(cross);
        if(Vector3DotProduct(n,Vector3Subtract(center,refInW))<0) n=Vector3Negate(n);
        float d=depth<s.maxDepth?depth:s.maxDepth;
        float Fmag=s.buoyDensity*area*d;
        Vector3 Fb=Vector3Scale(n,-Fmag);      // pressure force into the hull
        addF(Fb,center);
        // local drag at the facet (damps slap / oscillation)
        Vector3 r=Vector3Subtract(center,s.pos);
        Vector3 vAt=Vector3Add(s.vel,Vector3CrossProduct(s.angVel,r));
        Vector3 Ffd=Vector3Scale(vAt,-s.facetDrag*area);
        addF(Ffd,center);
        // record the TOTAL force applied at this sample point
        if(s.recordDebug) s.dbg.push_back({center,Vector3Add(Fb,Ffd),0});
    }
    float hullWet = totArea>1e-4f ? subArea/totArea : 0.0f;   // 0..1

    // ---- directional hull water resistance (only the submerged part) -------
    // Forward drag sets top speed; strong lateral (keel) resistance kills
    // side-slip so a rudder yaw makes the hull CARVE a turn instead of skidding.
    Vector3 up =Vector3RotateByQuaternion({0,1,0},s.orient);
    float vF=Vector3DotProduct(s.vel,fwd);
    float vS=Vector3DotProduct(s.vel,side);
    float vU=Vector3DotProduct(s.vel,up);
    Vector3 Fdf=Vector3Scale(fwd, -s.linDrag*vF*hullWet);
    Vector3 Fdl=Vector3Scale(side,-s.lateralDrag*std::fabs(vS)*vS*hullWet);
    Vector3 Fdh=Vector3Scale(up,  -s.heaveDamp*vU*hullWet);    // kills vertical bounce
    addF(Fdf,s.pos); addF(Fdl,s.pos); addF(Fdh,s.pos);
    torque=Vector3Subtract(torque,Vector3Scale(s.angVel,s.angDrag*hullWet));
    if(s.recordDebug) s.dbg.push_back({s.pos,Vector3Add(Vector3Add(Fdf,Fdl),Fdh),4});

    // smooth submersion factor for a point: full at >=0.3 m under, fading to
    // zero as it lifts ~0.3 m above the surface
    auto submersion=[&](Vector3 p)->float{
        float sub=oceanSampleHeight(o,p.x,p.z)-p.y;
        float f=(sub+0.3f)/0.6f;
        return f<0.0f?0.0f:(f>1.0f?1.0f:f);
    };

    // ---- motor thrust (only while the propeller is submerged) ----
    Vector3 propW=Vector3Add(s.pos,Vector3RotateByQuaternion(s.propPos,s.orient));
    float propF=submersion(propW);
    if(propF>0.0f){
        Vector3 Ft=Vector3Scale(fwd, s.thrustK*s.propOmega*propF);
        addF(Ft, propW);
        if(s.recordDebug) s.dbg.push_back({propW,Ft,2});
    }

    // ---- rudder: a flat plate reacting to the actual water flow over it -----
    // The plate normal (body space) rotates with the rudder deflection.  The
    // hydrodynamic force opposes the normal component of the plate's velocity
    // through the water (F = -0.5*rho*Cn*A * |v.n| v.n along n), applied at the
    // rudder.  So it only bites with real flow, scales with speed^2, induces
    // drag when turning, and adds into the total like any other force.
    Vector3 rudW=Vector3Add(s.pos,Vector3RotateByQuaternion(s.rudderPos,s.orient));
    float rudF=submersion(rudW);
    if(rudF>0.0f){
        Vector3 nBody={std::sin(s.rudderAngle),0.0f,std::cos(s.rudderAngle)};
        Vector3 nW=Vector3RotateByQuaternion(nBody,s.orient);
        Vector3 r=Vector3Subtract(rudW,s.pos);
        Vector3 vAt=Vector3Add(s.vel,Vector3CrossProduct(s.angVel,r));
        float vn=Vector3DotProduct(vAt,nW);
        float Fmag=s.rudderK*std::fabs(vn)*vn*rudF;
        Vector3 Fr=Vector3Scale(nW,-Fmag);
        addF(Fr, rudW);
        if(s.recordDebug) s.dbg.push_back({rudW,Fr,3});
    }

    // ---- integrate (semi-implicit Euler) ----
    Vector3 accel=Vector3Scale(force,1.0f/s.mass);
    s.vel=Vector3Add(s.vel,Vector3Scale(accel,dt));
    s.pos=Vector3Add(s.pos,Vector3Scale(s.vel,dt));

    // angular: world torque -> body -> /I -> world
    Quaternion qi=QuaternionInvert(s.orient);
    Vector3 tBody=Vector3RotateByQuaternion(torque,qi);
    Vector3 aBody={tBody.x*s.invInertiaDiag.x,
                   tBody.y*s.invInertiaDiag.y,
                   tBody.z*s.invInertiaDiag.z};
    Vector3 aWorld=Vector3RotateByQuaternion(aBody,s.orient);
    s.angVel=Vector3Add(s.angVel,Vector3Scale(aWorld,dt));

    Quaternion wq={s.angVel.x,s.angVel.y,s.angVel.z,0.0f};
    Quaternion dq=QuaternionScale(QuaternionMultiply(wq,s.orient),0.5f*dt);
    s.orient=QuaternionNormalize(QuaternionAdd(s.orient,dq));
}

void shipUnload(Ship& s){
    if(s.built){ UnloadModel(s.model); s.built=false; }
}
