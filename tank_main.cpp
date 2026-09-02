// =============================================================================
//  Tracked-tank simulator -- visualization / front-end.
//
//  A tank is a single Jolt box hull pushed by TWO tracks.  Each track is a row
//  of downward raycasts along the bottom of the hull (the road wheels); every
//  ray is a small tyre: spring+damper suspension load, a longitudinal thrust
//  from track-vs-ground slip, and a lateral grip force -- all capped by mu*Fz
//  (see phys::World::stepTank in physics.cpp).
//
//  The tank is "pushed by its wheels": an Engine -> torque converter -> 4-speed
//  AUTOMATIC -> final drive powertrain spins the sprockets, and
//  sprocketOmega*radius is the track SURFACE SPEED fed to the physics.  The
//  gearbox shifts itself on engine rpm, and 4th tops out at 60 km/h because the
//  torque curve falls to zero at the redline.  Steering is SKID-STEER -- the two
//  tracks get different drive, and the per-ray lateral grip turns that into yaw.
//  Hold a hard turn at rest and the inside track reverses for an in-place pivot.
//
//  Half the field is lowered to a flat lakebed under a flat water plane, joined
//  to the land by THREE shore lanes of different grade (gentle / medium /
//  steep).  Buoyancy is sampled at a grid of points through the hull envelope
//  (see phys::Buoy), so entering and leaving the water is a continuous handover
//  between suspension load and lift rather than a mode switch -- press B to see
//  the per-point forces.
//
//  Controls:  W/Up throttle   S/Down brake (hold at a standstill for reverse)
//             A/D (or arrows) skid-steer   R reset   P pause
//             B buoyancy debug arrows
//
//  Build:  cmake --build build --target tanksim   ->   ./build/tanksim
// =============================================================================
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "sim.h"
#include "physics.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdio>     // TEMP-AUTOTEST
#include <cstdlib>    // TEMP-AUTOTEST

// ----------------------------- water ----------------------------------------
// One flat water plane over the lowered half of the field.  The lakebed under
// it is flat too, so the tank can either swim or track along the bottom.
static const float LAKE_FLOOR = -4.0f;   // flat lakebed height [m]
static const float WATER_Y    = -1.5f;   // water surface  -> 2.5 m of water
static const float SHORE_X    = -6.0f;   // land edge: everything -X of this is shore/lake

// ----------------------------- tank test area -------------------------------
// LAND half (wx > SHORE_X): flat spawn pad, rows of round bumps of increasing
// height ahead, transverse "speed-bump" ridges to one side, deep pits to the
// other.  WATER half (wx < SHORE_X): the ground ramps down to a flat lakebed,
// with THREE shore lanes of different grade so the water can be entered and
// left at a gentle, a medium and a steep angle.
static float landH(float wx,float wz){
    float d = std::sqrt(wx*wx+wz*wz);
    if(d < 7.0f) return 0.0f;                    // flat spawn pad

    float h = 0.0f;
    auto bump=[&](float cx,float cz,float r,float amp){
        float dx=wx-cx, dz=wz-cz;
        h += amp*std::exp(-(dx*dx+dz*dz)/(2.0f*r*r));
    };

    // ahead (+X): three rows of round bumps of increasing height, then a mound
    bump(14,-3,1.5f,0.30f); bump(14,0,1.5f,0.30f); bump(14,3,1.5f,0.30f);
    bump(20,-3,1.6f,0.55f); bump(20,0,1.6f,0.55f); bump(20,3,1.6f,0.55f);
    bump(27,-3,1.8f,0.90f); bump(27,0,1.8f,0.90f); bump(27,3,1.8f,0.90f);
    bump(35, 0,2.4f,1.40f);                       // big mound

    // right (+Z): DEEP pits, getting deeper
    bump(12,12,2.0f,-1.0f);
    bump(20,14,2.2f,-1.7f);
    bump(28,16,2.6f,-2.4f);                       // very deep crater

    // left (-Z): transverse speed-bump ridges (kept clear of the shore)
    if(wz<-8.0f && wz>-22.0f && wx>-4.0f && wx<12.0f)
        h += 0.32f*(0.5f+0.5f*std::sin(wx*1.4f));  // ~4.5 m period ridges

    return h;
}

// Horizontal RUN of the shore ramp at this Z.  The drop is always the same
// (land level -> LAKE_FLOOR), so a short run means a steep slope.  Three
// plateaus give the three lanes; between them the run collapses to a cliff, so
// the lanes read as separate ramps instead of one continuous beach.
static float shoreRun(float wz){
    // Runs are chosen from the PEAK gradient, not the average: the smoothstep
    // below makes the steepest point 1.5x the mean, so a 4 m drop over a run R
    // tops out at atan(1.5*4/R).  That gives ~12 deg / ~22 deg / ~33 deg -- all
    // three climbable by a tracked vehicle, with the cliff (run 2) impassable.
    static const float zs[]  = {-40,-30,-24,-12, -6,  6, 12, 20, 26, 40};
    static const float runs[]= {  2,  2, 28, 28, 15, 15,  9,  9,  2,  2};
    const int n = 10;
    if(wz<=zs[0])   return runs[0];
    if(wz>=zs[n-1]) return runs[n-1];
    for(int i=1;i<n;i++) if(wz<=zs[i]){
        float t=(wz-zs[i-1])/(zs[i]-zs[i-1]);
        return runs[i-1] + (runs[i]-runs[i-1])*t;
    }
    return runs[n-1];
}

static float terrainH(float wx,float wz){
    if(wx >= SHORE_X) return landH(wx,wz);

    float top = landH(SHORE_X,wz);               // land height at the shore line
    float run = shoreRun(wz);
    float t   = (SHORE_X-wx)/run;                // 0 at the shore, 1 at the bottom
    if(t >= 1.0f) return LAKE_FLOOR;             // flat lakebed
    float e = t*t*(3.0f-2.0f*t);                 // smoothstep: no hard lip
    return top*(1.0f-e) + LAKE_FLOOR*e;
}

static int surfZone(float,float){ return 0; }   // uniform dry test ground
// Terrain colour is keyed off HEIGHT so the lakebed and the wet strip either
// side of the waterline read as mud and sand instead of grass.
static Color zoneColor(float wx,float wz,float h){
    (void)wx; (void)wz;
    if(h < WATER_Y - 0.15f) return Color{ 64, 60, 44,255};   // submerged mud
    if(h < WATER_Y + 0.35f) return Color{116,106, 76,255};   // wet sand at the line
    if(h < WATER_Y + 1.20f) return Color{ 92, 92, 66,255};   // dry shore
    return Color{68,80,62,255};                              // grass
}

static const char* TERRAIN_VS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec3 vertexNormal;\n"
    "in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matNormal;\n"
    "out vec3 fragNormal;\n"
    "out vec4 fragColor;\n"
    "void main(){\n"
    "    fragColor  = vertexColor;\n"
    "    fragNormal = normalize(vec3(matNormal*vec4(vertexNormal,0.0)));\n"
    "    gl_Position = mvp*vec4(vertexPosition,1.0);\n"
    "}\n";
static const char* TERRAIN_FS =
    "#version 330\n"
    "in vec3 fragNormal;\n"
    "in vec4 fragColor;\n"
    "uniform vec3 lightDir;\n"
    "uniform vec3 lightColor;\n"
    "uniform float ambient;\n"
    "uniform vec4 colDiffuse;\n"
    "out vec4 finalColor;\n"
    "void main(){\n"
    "    vec3 N = normalize(fragNormal);\n"
    "    float d = max(dot(N, -normalize(lightDir)), 0.0);\n"
    "    vec3 base = fragColor.rgb*colDiffuse.rgb;\n"
    "    vec3 lit  = base*(ambient + d*lightColor);\n"
    "    finalColor = vec4(lit, fragColor.a*colDiffuse.a);\n"
    "}\n";

static Model BuildTerrainModel(int N,float cell,const std::vector<float>& h){
    Mesh m{};
    m.vertexCount   = N*N;
    m.triangleCount = (N-1)*(N-1)*2;
    m.vertices = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.normals  = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.colors   = (unsigned char*)MemAlloc(m.vertexCount*4*sizeof(unsigned char));
    m.indices  = (unsigned short*)MemAlloc(m.triangleCount*3*sizeof(unsigned short));
    float span=(N-1)*cell;
    auto H=[&](int ix,int iz){ ix=ix<0?0:(ix>=N?N-1:ix); iz=iz<0?0:(iz>=N?N-1:iz);
                               return h[(size_t)iz*N+ix]; };
    for(int iz=0;iz<N;iz++) for(int ix=0;ix<N;ix++){
        int v=iz*N+ix;
        float wx=-span*0.5f+cell*ix, wz=-span*0.5f+cell*iz;
        m.vertices[v*3+0]=wx;
        m.vertices[v*3+1]=h[(size_t)iz*N+ix];
        m.vertices[v*3+2]=wz;
        float dx=H(ix+1,iz)-H(ix-1,iz);
        float dz=H(ix,iz+1)-H(ix,iz-1);
        Vector3 nrm=Vector3Normalize({-dx, 2.0f*cell, -dz});
        m.normals[v*3+0]=nrm.x; m.normals[v*3+1]=nrm.y; m.normals[v*3+2]=nrm.z;
        Color zc=zoneColor(wx,wz,h[(size_t)iz*N+ix]);
        m.colors[v*4+0]=zc.r; m.colors[v*4+1]=zc.g; m.colors[v*4+2]=zc.b; m.colors[v*4+3]=255;
    }
    int t=0;
    for(int iz=0;iz<N-1;iz++) for(int ix=0;ix<N-1;ix++){
        unsigned short a=iz*N+ix, b=iz*N+ix+1, c=(iz+1)*N+ix, d=(iz+1)*N+ix+1;
        m.indices[t++]=a; m.indices[t++]=c; m.indices[t++]=b;
        m.indices[t++]=b; m.indices[t++]=c; m.indices[t++]=d;
    }
    UploadMesh(&m,false);
    return LoadModelFromMesh(m);
}

// flat water quad at `y`, drawn translucent LAST so submerged geometry shows
// through it correctly.
static Model BuildWaterModel(float x0,float x1,float z0,float z1,float y){
    Mesh m{};
    m.vertexCount=4; m.triangleCount=2;
    m.vertices=(float*)MemAlloc(4*3*sizeof(float));
    m.normals =(float*)MemAlloc(4*3*sizeof(float));
    m.colors  =(unsigned char*)MemAlloc(4*4*sizeof(unsigned char));
    m.indices =(unsigned short*)MemAlloc(6*sizeof(unsigned short));
    float vx[4]={x0,x1,x0,x1}, vz[4]={z0,z0,z1,z1};
    for(int i=0;i<4;i++){
        m.vertices[i*3+0]=vx[i]; m.vertices[i*3+1]=y; m.vertices[i*3+2]=vz[i];
        m.normals [i*3+0]=0;     m.normals [i*3+1]=1; m.normals [i*3+2]=0;
        m.colors[i*4+0]=90; m.colors[i*4+1]=150; m.colors[i*4+2]=185; m.colors[i*4+3]=255;
    }
    unsigned short idx[6]={0,2,1, 1,2,3};
    for(int i=0;i<6;i++) m.indices[i]=idx[i];
    UploadMesh(&m,false);
    return LoadModelFromMesh(m);
}

// shaft + cone arrow, used for the buoyancy debug forces
static void drawArrow(Vector3 from, Vector3 dir, float len, float thick, Color c){
    if(len<0.02f) return;
    float dl=Vector3Length(dir);
    if(dl<1e-6f) return;
    Vector3 d   = Vector3Scale(dir,1.0f/dl);
    Vector3 tip = Vector3Add(from,Vector3Scale(d,len));
    Vector3 neck= Vector3Add(from,Vector3Scale(d,len*0.75f));
    DrawCylinderEx(from,neck,thick,thick,8,c);
    DrawCylinderEx(neck,tip,thick*2.8f,0.0f,10,c);
}

// ----------------------------- orbit camera ---------------------------------
static Vector3 gCamPos = {8.0f,7.0f,10.0f};
static float   gOrbitYaw=2.4f, gOrbitPitch=0.55f, gOrbitDist=16.0f;

// ----------------------------- tank layout ----------------------------------
static const int   ROADW   = 5;        // road wheels per track
static const float TRACK_Z = 1.0f;     // track lateral offset from centre [m]
static const float WHEEL_X0 = -1.0f, WHEEL_X1 = 1.0f;   // road-wheel X span
static const float ATTACH_Y = -0.4f;   // suspension-top height (body space)
static const float WHEEL_R  = 0.22f;   // road-wheel radius (visual + physics)
// drive sprocket (front) / idler (rear): pushed past the end wheels, raised a
// little above the road wheels so the belt wraps around them and the top run
// hugs under the hull.
static const float SPRO_X = 1.4f,  SPRO_Y = -0.62f;
static const float IDLE_X =-1.4f,  IDLE_Y = -0.62f;
static const float SPRO_R = 0.20f;     // sprocket/idler radius
static const float RAY_LEN = 0.64f;    // rest+radius (matches TrackSusp below)

// hull box (small body sitting between the tracks)
static const float HULL_L = 3.0f, HULL_H = 0.9f, HULL_W = 1.8f;

// Amphibious hull (PT-76 class): light enough that the displaced envelope --
// hull + sponsons + both track runs -- can actually carry it.
//
// TANK_VOL has to stay honest about the envelope the buoyancy points actually
// span (3.48 x 1.44 x 2.59 m = 13.0 m^3, see buildTank); a little over that is
// fair for a sealed hull and trim vane, a lot over it just makes the tank ride
// on top of the water like a cork.  14.5 m^3 against 12 t gives a 12/14.5 =
// 83% draft: it swims down in the water with the deck ~0.2 m clear, and keeps
// ~35% reserve buoyancy in the upper sample layer so a wave cannot swamp it.
static const float TANK_MASS = 12000.0f;   // [kg]
static const float TANK_VOL  = 14.5f;      // displaced volume fully under [m^3]

// ---------------------- powertrain: 4-speed automatic -----------------------
// Engine -> torque converter -> 4-speed auto -> final drive -> drive sprocket.
// The sprocket's surface speed (omega * SPRO_R) is what stepTank() wants, so
// the powertrain state is ONE number -- the sprocket speed -- integrated from
// the engine torque minus the track's reaction torque.  That reaction comes
// straight back out of the physics (phys::TrackOut::force), so the loop is
// closed: lose grip and the sprocket runs away, climb a hill and it bogs down.
static const double ENG_IDLE_RPM  = 700.0;    // converter never lets it stall
static const double ENG_REDLINE   = 2600.0;
static const double ENG_INERTIA   = 1.2;      // flywheel [kg m^2]
static const double TRK_INERTIA   = 60.0;     // sprockets + belt, at the sprocket
static const double DRIVE_EFF     = 0.90;
static const double FINAL_DRIVE   = 3.27;     // sized so 4th tops out at 60 km/h
static const double GEARS[4]      = {3.60, 2.10, 1.35, 1.00};
static const double REVERSE_RATIO = -3.60;
static const double STALL_RATIO   = 2.0;      // converter torque multiplication
static const double ENG_BRAKE     = 260.0;    // closed-throttle drag at redline [Nm]
static const double BRAKE_TORQUE  = 9000.0;   // sprocket brake at full pedal [Nm]
// The shift window has to clear the ratio steps.  An upshift drops the engine
// by ratio[g+1]/ratio[g] -- worst case 1->2, a 42% fall from 2350 to ~1370 rpm.
// A 1250 rpm downshift line sits inside that, so every upshift immediately
// bounced back down and the box hunted; at 0.35 s of open clutch per shift that
// left the engine disconnected most of the time and capped the tank at 23 km/h.
// 950 rpm clears the worst post-upshift landing with room to spare, and the
// hold timer stops any shift from chaining into another.
static const double UPSHIFT_RPM   = 2350.0;
static const double DOWNSHIFT_RPM =  950.0;
static const double SHIFT_TIME    = 0.35;     // clutch open this long per shift [s]
static const double SHIFT_HOLD    = 1.00;     // minimum time between shifts [s]

// ~300 hp turbo-diesel; the fall to zero at the redline IS the rev limiter, and
// is what caps top speed in 4th (nothing else resists it on flat ground).
static double engineTorque(double rpm){
    static const std::vector<double> rs{ 700, 1200, 1600, 2000, 2400, 2500, 2600};
    static const std::vector<double> ts{ 700, 1000, 1150, 1050,  850,  600,    0};
    return interp(rs, ts, clampd(rpm, rs.front(), rs.back()));
}

// body-space -> world helper
static Vector3 toWorld(Vector3 pos, Quaternion q, Vector3 local){
    return Vector3Add(pos, Vector3RotateByQuaternion(local,q));
}

// draw one small oriented track shoe (a flat box) at p, local frame X=dir,
// Y=up, Z=cross (across the track).
static void drawShoe(Vector3 p, Vector3 dir, Vector3 up, Vector3 crs,
                     Vector3 size, Color fill, Color wire){
    Matrix m = { dir.x, up.x, crs.x, p.x,
                 dir.y, up.y, crs.y, p.y,
                 dir.z, up.z, crs.z, p.z,
                 0,     0,    0,     1 };
    float16 mf = MatrixToFloatV(m);
    rlPushMatrix();
    rlMultMatrixf(mf.v);
    DrawCube({0,0,0}, size.x, size.y, size.z, fill);
    DrawCubeWires({0,0,0}, size.x, size.y, size.z, wire);
    rlPopMatrix();
}

// walk the belt polyline by ARC LENGTH, dropping a shoe every `spacing`.
// `phase` [m] scrolls the shoes along the belt so the track visibly moves.
static void drawTrackLinks(const std::vector<Vector3>& pts, Vector3 up,
                           Vector3 crs, float shoeW, float phase){
    const float spacing = 0.16f;
    Color fill{34,36,42,255}, wire{80,84,94,255};
    Vector3 size{0.10f, 0.07f, shoeW};
    // first shoe sits `off` in [0,spacing) along the path (the scroll offset)
    float off = phase - std::floor(phase/spacing)*spacing;
    float dist = off;
    for(size_t i=1;i<pts.size();++i){
        Vector3 a=pts[i-1], b=pts[i];
        Vector3 seg=Vector3Subtract(b,a);
        float len=Vector3Length(seg);
        if(len<1e-5f) continue;
        Vector3 dir=Vector3Scale(seg,1.0f/len);
        while(dist<len){
            drawShoe(Vector3Add(a,Vector3Scale(dir,dist)), dir, up, crs, size, fill, wire);
            dist+=spacing;
        }
        dist-=len;
    }
}

// one track-pad footprint left on the ground
struct Mark { Vector3 p; Vector3 dir; unsigned char a; };
static const float PAD_PITCH = 0.30f;   // ground spacing between pad prints [m]
static const float PAD_LEN   = 0.15f;   // length of one pad print [m] (< pitch -> gaps)
static const float PAD_WIDTH = 0.40f;   // pad print width (track width) [m]

int main(){
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1280,720,"tanksim -- tracked tank (raycast tracks)");
    SetTargetFPS(60);

    // ---- terrain / heightfield --------------------------------------------
    const int   TN=256;            // Jolt HeightField u16 index limit
    const float TCELL=0.6f;        // fine cells so the test bumps resolve (~153 m sq)
    const float TSPAN=(TN-1)*TCELL;
    std::vector<float> heights((size_t)TN*TN);
    for(int iz=0;iz<TN;iz++) for(int ix=0;ix<TN;ix++){
        float wx=-TSPAN*0.5f+TCELL*ix, wz=-TSPAN*0.5f+TCELL*iz;
        heights[(size_t)iz*TN+ix]=terrainH(wx,wz);
    }
    float spawnH = heights[(size_t)(TN/2)*TN + (TN/2)] + 2.0f;

    phys::World world;
    world.setHeightfield(TN,TCELL,heights);
    Model terrain = BuildTerrainModel(TN,TCELL,heights);

    Shader litShader = LoadShaderFromMemory(TERRAIN_VS,TERRAIN_FS);
    {
        Vector3 ld = Vector3Normalize({-0.55f,-1.0f,-0.35f});
        Vector3 lc = {1.0f,0.96f,0.88f};
        float   amb= 0.38f;
        SetShaderValue(litShader,GetShaderLocation(litShader,"lightDir"),  &ld,SHADER_UNIFORM_VEC3);
        SetShaderValue(litShader,GetShaderLocation(litShader,"lightColor"),&lc,SHADER_UNIFORM_VEC3);
        SetShaderValue(litShader,GetShaderLocation(litShader,"ambient"),   &amb,SHADER_UNIFORM_FLOAT);
    }
    terrain.materials[0].shader = litShader;

    // ---- build the tank ----------------------------------------------------
    phys::TrackSusp ts;
    ts.radius = WHEEL_R;   // small road wheels (just touching)
    ts.rest   = 0.42f;     // -> RAY_LEN = rest+radius = 0.64
    ts.travel = 0.28f;
    // springs sized for the lighter amphibious hull: 12 t over 10 rays is
    // 1.2 t each, and ~200 kN/m puts that at a third of the usable travel.
    ts.stiffness = 200000.0f;
    ts.damping   =  22000.0f;
    world.setTrackSusp(ts);
    world.buildTank(ROADW, WHEEL_X0, WHEEL_X1, -TRACK_Z, +TRACK_Z, ATTACH_Y,
                    HULL_L, HULL_H, HULL_W, TANK_MASS,
                    0.0f, spawnH, 0.0f,
                    0.0f, -0.4f, 0.0f /*low COM for stability*/);

    // ---- water --------------------------------------------------------
    phys::Buoy bo;
    bo.level  = WATER_Y;
    bo.volume = TANK_VOL;
    world.setBuoy(bo);
    Model water = BuildWaterModel(-TSPAN*0.5f, SHORE_X+1.0f,
                                  -TSPAN*0.5f,  TSPAN*0.5f, WATER_Y);
    water.materials[0].shader = litShader;

    // a climb ramp past the bump field, plus a few crates to push around
    world.addRamp(46.0f,0.0f,0.0f,0.0f, 10.0f,2.0f,7.0f);
    world.addCrate(6.0f,0.8f,-4.0f,0.7f,60.0f);
    world.addCrate(6.0f,0.8f,-5.6f,0.7f,60.0f);
    world.addCrate(7.4f,0.8f,-4.8f,0.7f,60.0f);

    // ---- two-track control -------------------------------------------------
    // Each track is commanded a SURFACE SPEED [m/s]; the raycast slip model in
    // stepTank turns that into grip-limited thrust.  The powertrain sets the
    // BASE speed both tracks share; the steering unit adds a differential on
    // top (hold a turn at rest to pivot), exactly as a real tank does it.
    const float TURN_SPEED= 3.5f;    // differential added by a full turn [m/s]
    double omegaS = 0.0;             // drive-sprocket speed [rad/s] -- powertrain state
    int    gear   = 1;               // 1..4 forward, -1 reverse
    double shiftT = 0.0;             // remaining clutch-open time [s]
    double holdT  = 0.0;             // lockout before the next shift is allowed [s]
    double rpm    = ENG_IDLE_RPM;    // for the HUD
    float  throttle=0, brakeCmd=0;   // ramped pedals
    float  turnCmd=0;                // ramped steering
    float leftSurf=0, rightSurf=0;   // for HUD
    float trackPhase[2]={0,0};       // belt scroll offset per side [m]
    std::vector<Mark> marks[2];      // skid-mark ribbon points per side
    Vector3 lastMark[2]={{1e9f,0,0},{1e9f,0,0}};
    const size_t MARK_CAP=900;
    bool  running=true;
    bool  showBuoy=true;             // B: buoyancy sample points + force arrows
    // debug arrow scaling: metres of arrow per newton of force
    const float BUOY_SCALE = 1.0f/30000.0f;   // 30 kN -> 1 m
    const float DRAG_SCALE = 1.0f/30000.0f;

    while(!WindowShouldClose()){
        // ---- input: throttle / brake / steering ----------------------------
        float thrTgt=0.0f, brkTgt=0.0f, turnTgt=0.0f;
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))    thrTgt  = 1.0f;
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN))  brkTgt  = 1.0f;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) turnTgt += 1.0f;
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))  turnTgt -= 1.0f;
        // TEMP-AUTOTEST: full throttle along z=40, which is clear flat land
        static int autoTest = 0;
        static int atFrame  = -1;
        static bool atInit  = false;
        if(!atInit){ atInit=true;
                     const char* e=getenv("TANK_AUTOTEST"); if(e) autoTest=atoi(e); }
        if(autoTest){
            if(atFrame<0) world.resetVehicle(-5.0f, spawnH, 40.0f);
            thrTgt=1.0f; brkTgt=0.0f; turnTgt=0.0f; atFrame++;
        }
        throttle += (thrTgt-throttle)*0.15f;   // pedals ramp, no lurch
        brakeCmd += (brkTgt-brakeCmd)*0.25f;
        turnCmd  += (turnTgt-turnCmd)*0.08f;
        if(IsKeyPressed(KEY_P)) running=!running;
        if(IsKeyPressed(KEY_B)) showBuoy=!showBuoy;
        if(IsKeyPressed(KEY_R)){ world.resetVehicle(0,spawnH,0);
                                 world.resetObstacles();
                                 throttle=brakeCmd=turnCmd=0;
                                 omegaS=0; gear=1; shiftT=0; holdT=0;
                                 marks[0].clear(); marks[1].clear();
                                 lastMark[0]=lastMark[1]=Vector3{1e9f,0,0}; }

        // ---- physics -------------------------------------------------------
        float mu = 0.9f;
        if(running && world.hasVehicle()){
            float bp0[3]; world.bodyPosition(bp0);
            mu = (float)SURFACES[surfZone(bp0[0],bp0[2])].mu;

            const double dt = 1.0/60.0;

            // ---- range select (D <-> R) ------------------------------------
            // An automatic swaps range when you hold the opposing pedal and the
            // driveline has already stopped -- so S brakes you to a halt first,
            // then becomes reverse, and W pulls it back into drive.
            if(holdT>0.0) holdT -= dt;
            if(gear==-1){
                if(thrTgt>0.5f && std::fabs(omegaS)<2.0){
                    gear=1; shiftT=SHIFT_TIME; holdT=SHIFT_HOLD; }
            } else if(brkTgt>0.5f && std::fabs(omegaS)<2.0
                                  && std::fabs(world.forwardSpeed())<0.5f){
                gear=-1; shiftT=SHIFT_TIME; holdT=SHIFT_HOLD;
            }
            double thr = (gear==-1) ? brakeCmd : throttle;   // in R the pedals swap
            double brk = (gear==-1) ? throttle : brakeCmd;

            // ---- engine + torque converter ---------------------------------
            double nTot  = ((gear==-1) ? REVERSE_RATIO : GEARS[gear-1])*FINAL_DRIVE;
            double wTurb = omegaS*nTot;                       // turbine (gearbox input)
            double wEng  = std::max(ENG_IDLE_RPM*RPM2RAD, wTurb);
            rpm          = wEng*RAD2RPM;
            // slip across the converter multiplies torque at a standstill (that
            // is the launch) and fades to 1:1 once the turbine catches up
            double sr    = clampd(wTurb/std::max(1e-3,wEng), 0.0, 1.0);
            double tqR   = 1.0 + (STALL_RATIO-1.0)*(1.0-sr);
            double Te    = thr*engineTorque(rpm)
                         - (1.0-thr)*ENG_BRAKE*(wEng/(ENG_REDLINE*RPM2RAD));
            if(shiftT>0.0){ shiftT -= dt; Te = 0.0; }         // clutch open mid-shift

            // ---- sprocket: engine torque vs the track reaction --------------
            double Ftrk = world.track(0).force + world.track(1).force;  // last step
            double J    = ENG_INERTIA*nTot*nTot + TRK_INERTIA;
            omegaS += (Te*tqR*nTot*DRIVE_EFF
                       - Ftrk*SPRO_R
                       - brk*BRAKE_TORQUE*sgn(omegaS))*dt/J;
            if(brk>0.5 && std::fabs(omegaS)<0.5) omegaS = 0.0;   // no standstill jitter

            // ---- shift schedule (purely rpm-based) --------------------------
            if(shiftT<=0.0 && holdT<=0.0 && gear>=1){
                if     (rpm>UPSHIFT_RPM   && gear<4){
                    gear++; shiftT=SHIFT_TIME; holdT=SHIFT_HOLD; }
                else if(rpm<DOWNSHIFT_RPM && gear>1){
                    gear--; shiftT=SHIFT_TIME; holdT=SHIFT_HOLD; }
            }

            // ---- steering unit ---------------------------------------------
            float base = (float)(omegaS*SPRO_R);
            // Authority tapers with speed: the full differential for a
            // standstill pivot, under half of it at 60 km/h, so a twitch at
            // speed cannot swap ends.
            float turnAuth = TURN_SPEED*(float)clampd(1.0-0.55*std::fabs(base)/16.7,
                                                      0.45, 1.0);
            leftSurf  = base + turnCmd*turnAuth;   // turn>0 (D): left faster
            rightSurf = base - turnCmd*turnAuth;

            // stopped and coasting -> static brake hold so it parks on slopes
            float brake = (brk>0.5 || (thr<0.05 && std::fabs(base)<0.15f
                                                && std::fabs(turnCmd)<0.05f)) ? 1.0f : 0.0f;

            world.stepTank((float)dt, leftSurf, rightSurf, brake, mu);

            // TEMP-AUTOTEST
            if(autoTest){
                if(atFrame%30==0){
                    float ap[3]; world.bodyPosition(ap);
                    printf("t=%5.1f  gear=%2d  rpm=%6.0f  track=%5.2f m/s  "
                           "speed=%5.1f km/h  x=%6.1f\n",
                           atFrame/60.0f, gear, rpm, (float)(omegaS*SPRO_R),
                           world.forwardSpeed()*3.6f, ap[0]);
                    fflush(stdout);
                }
                float ax[3]; world.bodyPosition(ax);
                if(ax[0] > 70.0f || atFrame > 60*45) break;
            }

            // scroll the visible track belts with each track's surface speed
            trackPhase[0] += leftSurf *(1.0f/60.0f);
            trackPhase[1] += rightSurf*(1.0f/60.0f);

            // lay down skid marks under each track (darker where it's slipping)
            {
                float mq[4]; world.bodyQuat(mq); Quaternion mqq={mq[0],mq[1],mq[2],mq[3]};
                Vector3 mUp   =Vector3RotateByQuaternion({0,1,0},mqq);
                Vector3 mFwd  =Vector3RotateByQuaternion({1,0,0},mqq);
                const auto& wm=world.wheels();
                for(int side=0;side<2;side++){
                    Vector3 sum={0,0,0}; float fzSum=0; int gc=0;
                    for(int k=0;k<ROADW;k++){ int i=side*ROADW+k;
                        if(i<(int)wm.size() && wm[i].grounded){
                            Vector3 c={wm[i].x,wm[i].y,wm[i].z};
                            sum=Vector3Add(sum,Vector3Subtract(c,Vector3Scale(mUp,ts.radius)));
                            fzSum+=wm[i].Fz; gc++; } }
                    if(gc==0) continue;
                    Vector3 mid=Vector3Scale(sum,1.0f/gc);
                    // drop a discrete PAD print every pad-pitch of travel: a real
                    // track pad sits at one ground spot as the hull rolls over it,
                    // leaving a separated footprint pattern (not a continuous line).
                    if(Vector3Distance(mid,lastMark[side])>PAD_PITCH){
                        Vector3 dir = (lastMark[side].x<1e8f)
                            ? Vector3Normalize(Vector3Subtract(mid,lastMark[side])) : mFwd;
                        float meanFz = fzSum/gc;                 // per-ray load [N]
                        unsigned char a=(unsigned char)clampd(165.0+meanFz/900.0, 165.0, 240.0);
                        mid.y += 0.02f;                  // sit the pad on the ground
                        marks[side].push_back({mid, dir, a});
                        if(marks[side].size()>MARK_CAP) marks[side].erase(marks[side].begin());
                        lastMark[side]=mid;
                    }
                }
            }

            // fell off the map? respawn at centre
            float bp[3]; world.bodyPosition(bp);
            float half=TSPAN*0.5f;
            if(bp[1]<-10.0f || std::fabs(bp[0])>half+4.0f || std::fabs(bp[2])>half+4.0f){
                world.resetVehicle(0,spawnH,0);
                throttle=brakeCmd=turnCmd=0;
                omegaS=0; gear=1; shiftT=0; holdT=0;
            }
        }

        // ---- camera --------------------------------------------------------
        float bp[3]; world.bodyPosition(bp);
        float bq[4]; world.bodyQuat(bq);
        Quaternion q={bq[0],bq[1],bq[2],bq[3]};
        Vector3 pos={bp[0],bp[1],bp[2]};
        Vector3 fwd  = Vector3RotateByQuaternion({1,0,0},q);
        Vector3 up   = Vector3RotateByQuaternion({0,1,0},q);
        Vector3 right= Vector3RotateByQuaternion({0,0,1},q);

        bool dragging = IsMouseButtonDown(MOUSE_BUTTON_LEFT)||IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
        if(dragging){
            Vector2 d=GetMouseDelta();
            gOrbitYaw   -= d.x*0.006f;
            gOrbitPitch += d.y*0.006f;
            gOrbitPitch  = (float)clampd(gOrbitPitch,0.08f,1.45f);
        } else {
            // third-person follow: slowly swing the orbit yaw to sit BEHIND the
            // tank (opposite its forward heading), via the shortest angle.
            float yawWant = std::atan2(-fwd.x, -fwd.z);
            float dyaw = yawWant - gOrbitYaw;
            while(dyaw >  PI) dyaw -= 2.0f*PI;         // wrap to [-PI, PI]
            while(dyaw < -PI) dyaw += 2.0f*PI;
            gOrbitYaw += dyaw*0.03f;                    // slow lerp
        }
        float wheel=GetMouseWheelMove();
        if(wheel!=0) gOrbitDist=(float)clampd(gOrbitDist-wheel*2.0f,6.0f,60.0f);
        Vector3 tgt = Vector3Add(pos,Vector3{0,0.6f,0});
        Vector3 off = { std::cos(gOrbitPitch)*std::sin(gOrbitYaw),
                        std::sin(gOrbitPitch),
                        std::cos(gOrbitPitch)*std::cos(gOrbitYaw) };
        Vector3 want= Vector3Add(tgt, Vector3Scale(off, gOrbitDist));
        gCamPos = Vector3Lerp(gCamPos, want, 0.25f);
        Camera3D cam{};
        cam.position=gCamPos; cam.target=tgt;
        cam.up={0,1,0}; cam.fovy=50.0f; cam.projection=CAMERA_PERSPECTIVE;

        // ---- render --------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{20,24,34,255});
        BeginMode3D(cam);

        DrawModel(terrain,{0,0,0},1.0f,WHITE);
        DrawModelWires(terrain,{0,0,0},1.0f,Color{255,255,255,30});

        // skid marks: discrete track-pad footprints, separated along the trail
        for(int side=0;side<2;side++){
            const std::vector<Mark>& mk=marks[side];
            for(size_t i=0;i<mk.size();++i){
                Vector3 dir=mk[i].dir, upW={0,1,0};
                Vector3 crs=Vector3Normalize(Vector3CrossProduct(dir,upW));
                Matrix m={ dir.x, upW.x, crs.x, mk[i].p.x,
                           dir.y, upW.y, crs.y, mk[i].p.y,
                           dir.z, upW.z, crs.z, mk[i].p.z,
                           0,     0,     0,     1 };
                Color col{14,12,10, mk[i].a};
                rlPushMatrix();
                rlMultMatrixf(MatrixToFloatV(m).v);
                DrawCube({0,0,0}, PAD_LEN, 0.05f, PAD_WIDTH, col);
                rlPopMatrix();
            }
        }

        // hull box
        float dims[3]; world.bodyDims(dims);
        rlPushMatrix();
        rlTranslatef(pos.x,pos.y,pos.z);
        Vector3 axis; float ang; QuaternionToAxisAngle(q,&axis,&ang);
        if(Vector3Length(axis)>0.001f) rlRotatef(ang*RAD2DEG,axis.x,axis.y,axis.z);
        DrawCube({0,0,0},dims[0],dims[1],dims[2],Color{70,86,64,255});
        DrawCubeWires({0,0,0},dims[0],dims[1],dims[2],Color{150,170,140,255});
        // a little turret cue
        DrawCube({0.1f,dims[1]*0.5f+0.20f,0},1.1f,0.4f,1.0f,Color{58,72,54,255});
        DrawCube({1.2f,dims[1]*0.5f+0.28f,0},1.6f,0.12f,0.12f,Color{40,50,38,255}); // barrel
        rlPopMatrix();

        // obstacles
        for(const phys::ObstacleOut& o : world.obstacles()){
            Quaternion oq={o.qx,o.qy,o.qz,o.qw};
            Vector3 op={o.px,o.py,o.pz};
            if(o.kind==0){
                rlPushMatrix();
                rlTranslatef(op.x,op.y,op.z);
                Vector3 ax; float an; QuaternionToAxisAngle(oq,&ax,&an);
                if(Vector3Length(ax)>0.001f) rlRotatef(an*RAD2DEG,ax.x,ax.y,ax.z);
                float s=o.sx*2.0f;
                DrawCube({0,0,0},s,s,s,Color{150,95,55,255});
                DrawCubeWires({0,0,0},s,s,s,Color{40,28,20,255});
                rlPopMatrix();
            } else {
                float hL=o.sx*0.5f, h=o.sy, hw=o.sz*0.5f;
                Vector3 lv[6]={{-hL,0,-hw},{-hL,0,hw},{hL,0,-hw},
                               {hL,0,hw},{hL,h,-hw},{hL,h,hw}};
                Vector3 wv[6];
                for(int k=0;k<6;k++) wv[k]=Vector3Add(op,Vector3RotateByQuaternion(lv[k],oq));
                Color fill={78,90,108,255};
                rlDisableBackfaceCulling();
                DrawTriangle3D(wv[0],wv[1],wv[5],fill); DrawTriangle3D(wv[0],wv[5],wv[4],fill);
                DrawTriangle3D(wv[0],wv[2],wv[3],fill); DrawTriangle3D(wv[0],wv[3],wv[1],fill);
                DrawTriangle3D(wv[2],wv[4],wv[5],fill); DrawTriangle3D(wv[2],wv[5],wv[3],fill);
                DrawTriangle3D(wv[0],wv[4],wv[2],fill); DrawTriangle3D(wv[1],wv[3],wv[5],fill);
                rlEnableBackfaceCulling();
            }
        }

        // tracks: raycasts + road wheels + track links (small shoes)
        const auto& wo = world.wheels();
        for(int side=0; side<2; ++side){
            float z = side==0 ? -TRACK_Z : +TRACK_Z;
            Vector3 spro = toWorld(pos,q,{ SPRO_X, SPRO_Y, z});
            Vector3 idle = toWorld(pos,q,{ IDLE_X, IDLE_Y, z});

            // raised sprocket (drive, front) and idler (rear) wheels
            Vector3 axw = Vector3Scale(right,0.14f);
            DrawCylinderEx(Vector3Subtract(spro,axw),Vector3Add(spro,axw),SPRO_R,SPRO_R,16,
                           Color{200,120,60,255});   // drive sprocket = warm
            DrawCylinderEx(Vector3Subtract(idle,axw),Vector3Add(idle,axw),SPRO_R,SPRO_R,16,
                           Color{60,64,74,255});      // idler = grey

            // draw rays + road wheels; collect each wheel's ground-contact point
            Vector3 wheelBottom[ROADW];
            for(int k=0;k<ROADW;k++){
                int i = side*ROADW + k;
                if(i>=(int)wo.size()) break;
                float rx = WHEEL_X0 + (WHEEL_X1-WHEEL_X0)*(ROADW>1?(float)k/(ROADW-1):0.5f);
                Vector3 attach = toWorld(pos,q,{ rx, ATTACH_Y, z});
                Vector3 c={wo[i].x,wo[i].y,wo[i].z};

                // --- raycast visualization ---
                Vector3 rayEnd = Vector3Add(attach, Vector3Scale(up, -RAY_LEN));
                DrawLine3D(attach, rayEnd, Color{70,80,95,140});      // full ray extent
                if(wo[i].grounded){
                    Vector3 contact = Vector3Subtract(c, Vector3Scale(up, ts.radius));
                    // load-coloured hit ray: green (light) -> red (heavy)
                    float load = std::min(1.0f, wo[i].Fz/80000.0f);
                    Color rc{ (unsigned char)(90+165*load),
                              (unsigned char)(220-150*load), 70, 255};
                    DrawLine3D(attach, contact, rc);
                    DrawSphere(contact, 0.07f, rc);
                } else {
                    DrawSphere(rayEnd, 0.05f, Color{240,160,90,255});  // airborne
                }

                // --- road wheel ---
                Vector3 a=Vector3Subtract(c,Vector3Scale(right,0.14f));
                Vector3 b=Vector3Add(c,Vector3Scale(right,0.14f));
                Color tc = wo[i].grounded?Color{36,40,48,255}:Color{90,70,40,255};
                DrawCylinderEx(a,b,ts.radius,ts.radius,14,tc);
                DrawCylinderWiresEx(a,b,ts.radius,ts.radius,14,Color{110,114,124,255});

                wheelBottom[k] = Vector3Subtract(c, Vector3Scale(up, ts.radius));
            }

            // belt loop that WRAPS AROUND the sprocket & idler:
            //   sprocket arc (top->front->bottom) -> ground run under the road
            //   wheels -> idler arc (bottom->rear->top) -> top run back.
            // arc point on a wheel at angle `a` in the fwd/up plane
            auto arc=[&](Vector3 ctr,float a){
                return Vector3Add(ctr, Vector3Add(Vector3Scale(fwd,std::cos(a)*SPRO_R),
                                                  Vector3Scale(up, std::sin(a)*SPRO_R)));
            };
            std::vector<Vector3> belt;
            // front sprocket: wrap the front half, top (+90) -> front (0) -> bottom (-90)
            for(int s=0;s<=6;s++){ float a=(float)(PI*0.5 - PI*s/6.0); belt.push_back(arc(spro,a)); }
            // ground run along the wheel bottoms, front -> rear
            for(int k=ROADW-1;k>=0;--k) belt.push_back(wheelBottom[k]);
            // rear idler: wrap the rear half, bottom (-90/270) -> rear (180) -> top (90)
            for(int s=0;s<=6;s++){ float a=(float)(-PI*0.5 - PI*s/6.0); belt.push_back(arc(idle,a)); }
            belt.push_back(belt.front());   // top run closes the loop

            // draw the track as many small shoe shapes; scroll with track speed
            drawTrackLinks(belt, up, right, 0.34f, trackPhase[side]);
        }

        // ---- buoyancy debug: sample points and the forces at them ----------
        // Each point is one of the 12 hull-envelope samples the physics uses.
        // GREEN = buoyant lift at that point, CYAN = water drag.  Watch them
        // light up front-to-back as the tank noses down a shore lane: that
        // staggered lift IS the terrain -> water transition.
        const auto& bps = world.buoyPoints();
        if(showBuoy){
            for(const phys::BuoyPoint& bpt : bps){
                Vector3 P{bpt.x,bpt.y,bpt.z};
                Color pc = bpt.sub>0.001f
                    ? Color{(unsigned char)(70+40*(1.0f-bpt.sub)), 170, 235, 255}
                    : Color{150,155,165,200};
                DrawSphere(P, 0.05f+0.05f*bpt.sub, pc);

                Vector3 fb{bpt.fx,bpt.fy,bpt.fz};
                drawArrow(P, fb, Vector3Length(fb)*BUOY_SCALE, 0.025f,
                          Color{90,235,120,255});
                Vector3 fd{bpt.dx,bpt.dy,bpt.dz};
                drawArrow(P, fd, Vector3Length(fd)*DRAG_SCALE, 0.020f,
                          Color{90,205,235,255});
            }
        }

        // water LAST, translucent, with depth writes off so the submerged hull
        // and the debug arrows stay visible through the surface
        rlDisableDepthMask();
        DrawModel(water,{0,0,0},1.0f,Color{70,135,175,140});
        rlEnableDepthMask();

        EndMode3D();

        // ---- HUD -----------------------------------------------------------
        float spd = world.forwardSpeed()*3.6f;

        // water readouts: how much of the hull is under, how much lift that is
        float subSum=0.0f, liftSum=0.0f;
        for(const phys::BuoyPoint& bpt : bps){ subSum+=bpt.sub; liftSum+=bpt.fy; }
        float subFrac = bps.empty() ? 0.0f : subSum/(float)bps.size();
        float weight  = TANK_MASS*9.81f;
        int   grounded= world.track(0).groundedRays + world.track(1).groundedRays;

        const char* gearTxt = (gear==-1) ? "R" : TextFormat("%d", gear);
        bool shifting = shiftT > 0.0;

        DrawRectangle(0,0,330,226,Color{0,0,0,150});
        DrawText(TextFormat("SPEED   %6.1f km/h", spd),            16,14,18,RAYWHITE);
        DrawText(TextFormat("GEAR  %2s / 4%s", gearTxt, shifting?"   shifting":""),
                                                                   16,42,18,
                 shifting?Color{240,180,90,255}:Color{200,230,255,255});
        DrawText(TextFormat("RPM   %6.0f  (redline %.0f)", rpm, ENG_REDLINE),
                                                                   16,70,16,
                 rpm>UPSHIFT_RPM?Color{240,140,120,255}:Color{170,200,240,255});
        // rpm bar: the shift schedule is easier to read as a bar than a number
        {
            float f = (float)clampd((rpm-ENG_IDLE_RPM)/(ENG_REDLINE-ENG_IDLE_RPM),0.0,1.0);
            float up= (float)((UPSHIFT_RPM-ENG_IDLE_RPM)/(ENG_REDLINE-ENG_IDLE_RPM));
            DrawRectangle(16,92,290,8,Color{40,46,58,255});
            DrawRectangle(16,92,(int)(290*f),8,
                          rpm>UPSHIFT_RPM?Color{230,110,90,255}:Color{110,200,140,255});
            DrawRectangle(16+(int)(290*up),89,2,14,Color{240,210,110,255});   // upshift mark
        }
        DrawText(TextFormat("TRACK L %5.1f  R %5.1f m/s", leftSurf, rightSurf),
                                                                   16,108,16,Color{170,200,240,255});
        DrawText(TextFormat("FORCE L %6.1f R %6.1f kN",
                 world.track(0).force/1000.0f, world.track(1).force/1000.0f),
                                                                   16,130,16,Color{150,160,175,255});
        DrawText(TextFormat("HULL    %3.0f %% submerged", subFrac*100.0f),
                                                                   16,152,16,Color{120,200,240,255});
        DrawText(TextFormat("LIFT    %6.1f kN  (weight %5.1f)",
                 liftSum/1000.0f, weight/1000.0f),                 16,174,16,Color{120,220,150,255});
        // which regime the tank is actually in -- the thing the transition is about
        const char* mode = (grounded==0 && subFrac>0.05f) ? "SWIMMING (track paddle)"
                         : (subFrac>0.05f)                ? "WADING (tracks on bed)"
                                                          : "ON LAND";
        DrawText(mode, 16,196,16,Color{240,215,120,255});
        if(!running) DrawText("PAUSED (P)", 200,196,16,Color{240,200,90,255});

        DrawText(TextFormat("W throttle   S brake (hold when stopped = reverse)   "
                            "A/D turn   R reset   P pause   B debug [%s]",
                            showBuoy?"on":"off"),
                 16, GetScreenHeight()-26, 16, Color{120,128,140,255});
        if(showBuoy)
            DrawText("green = buoyant lift   cyan = water drag   (1 m arrow = 30 kN)",
                     16, GetScreenHeight()-48, 15, Color{110,150,170,255});

        EndDrawing();
    }

    UnloadShader(litShader);
    UnloadModel(water);
    UnloadModel(terrain);
    CloseWindow();
    return 0;
}
