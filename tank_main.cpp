// =============================================================================
//  Tracked-tank simulator -- visualization / front-end.
//
//  A tank is a single Jolt box hull pushed by TWO tracks.  Each track is a row
//  of downward raycasts along the bottom of the hull (the road wheels); every
//  ray is a small tyre: spring+damper suspension load, a longitudinal thrust
//  from track-vs-ground slip, and a lateral grip force -- all capped by mu*Fz
//  (see phys::World::stepTank in physics.cpp).
//
//  The tank is "pushed by its wheels": an Engine -> Clutch -> Gearbox powertrain
//  (reused from sim.h) spins two sprockets; sprocketOmega*radius is the track
//  SURFACE SPEED fed to the physics.  Steering is SKID-STEER -- the two tracks
//  get different drive, and the per-ray lateral grip turns that into yaw.  Hold
//  a hard turn at rest and the inside track reverses for an in-place pivot.
//
//  Controls:  W/Up throttle   S/Down brake   A/D (or arrows) skid-steer
//             Q/E gear down/up   Space clutch   R reset   P pause
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

// ----------------------------- tank test area -------------------------------
// Flat base ground with a deliberate obstacle course: a flat spawn pad, rows of
// round bumps of increasing height ahead, transverse "speed-bump" ridges to one
// side, and a set of DEEP pits/trench to the other side.
static float terrainH(float wx,float wz){
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

    // left (-Z): transverse speed-bump ridges
    if(wz<-8.0f && wz>-22.0f && wx>-12.0f && wx<12.0f)
        h += 0.32f*(0.5f+0.5f*std::sin(wx*1.4f));  // ~4.5 m period ridges

    // behind (-X): a long deep trench across the field
    if(wx<-10.0f && wx>-16.0f)
        h -= 1.9f*(float)clampd(1.0 - std::fabs(wz)/14.0, 0.0, 1.0);

    return h;
}
static int surfZone(float,float){ return 0; }   // uniform dry test ground
static Color zoneColor(int){ return Color{68,80,62,255}; }

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
        Color zc=zoneColor(surfZone(wx,wz));
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
    world.setTrackSusp(ts);
    world.buildTank(ROADW, WHEEL_X0, WHEEL_X1, -TRACK_Z, +TRACK_Z, ATTACH_Y,
                    HULL_L, HULL_H, HULL_W, 40000.0f/*mass*/,
                    0.0f, spawnH, 0.0f,
                    0.0f, -0.4f, 0.0f /*low COM for stability*/);

    // a climb ramp past the bump field, plus a few crates to push around
    world.addRamp(46.0f,0.0f,0.0f,0.0f, 10.0f,2.0f,7.0f);
    world.addCrate(6.0f,0.8f,-4.0f,0.7f,60.0f);
    world.addCrate(6.0f,0.8f,-5.6f,0.7f,60.0f);
    world.addCrate(7.4f,0.8f,-4.8f,0.7f,60.0f);

    // ---- direct two-track control (no engine/gearbox) ----------------------
    // Each track is commanded a SURFACE SPEED [m/s]; the raycast slip model in
    // stepTank turns that into grip-limited thrust.  Forward/back sets both
    // tracks; left/right adds a differential (hold a turn at rest to pivot).
    const float MAX_SPEED = 5.0f;    // top track speed [m/s]
    const float TURN_SPEED= 3.5f;    // differential added by a full turn [m/s]
    float fwdCmd=0, turnCmd=0;       // ramped commands
    float leftSurf=0, rightSurf=0;   // for HUD
    float trackPhase[2]={0,0};       // belt scroll offset per side [m]
    std::vector<Mark> marks[2];      // skid-mark ribbon points per side
    Vector3 lastMark[2]={{1e9f,0,0},{1e9f,0,0}};
    const size_t MARK_CAP=900;
    bool  running=true;

    while(!WindowShouldClose()){
        // ---- input: forward/back + left/right ------------------------------
        float fwdTgt=0.0f, turnTgt=0.0f;
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))    fwdTgt += MAX_SPEED;
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN))  fwdTgt -= MAX_SPEED;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) turnTgt += 1.0f;
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))  turnTgt -= 1.0f;
        fwdCmd  += (fwdTgt -fwdCmd )*0.04f;    // gentle ramp (no lurch)
        turnCmd += (turnTgt-turnCmd)*0.08f;
        if(IsKeyPressed(KEY_P)) running=!running;
        if(IsKeyPressed(KEY_R)){ world.resetVehicle(0,spawnH,0);
                                 world.resetObstacles();
                                 fwdCmd=turnCmd=0;
                                 marks[0].clear(); marks[1].clear();
                                 lastMark[0]=lastMark[1]=Vector3{1e9f,0,0}; }

        // ---- physics -------------------------------------------------------
        float mu = 0.9f;
        if(running && world.hasVehicle()){
            float bp0[3]; world.bodyPosition(bp0);
            mu = (float)SURFACES[surfZone(bp0[0],bp0[2])].mu;

            leftSurf  = fwdCmd + turnCmd*TURN_SPEED;   // turn>0 (D): left faster
            rightSurf = fwdCmd - turnCmd*TURN_SPEED;

            // idle (no command) -> engage the static brake hold so it parks on slopes
            float brake = (std::fabs(fwdCmd)<0.15f && std::fabs(turnCmd)<0.05f) ? 1.0f : 0.0f;

            world.stepTank(1.0f/60.0f, leftSurf, rightSurf, brake, mu);

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
                fwdCmd=turnCmd=0;
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

        EndMode3D();

        // ---- HUD -----------------------------------------------------------
        float spd = world.forwardSpeed()*3.6f;
        DrawRectangle(0,0,300,120,Color{0,0,0,150});
        DrawText(TextFormat("SPEED   %6.1f km/h", spd),            16,14,18,RAYWHITE);
        DrawText(TextFormat("TRACK L %5.1f  R %5.1f m/s", leftSurf, rightSurf),
                                                                   16,42,18,Color{170,200,240,255});
        DrawText(TextFormat("FORCE L %6.1f R %6.1f kN",
                 world.track(0).force/1000.0f, world.track(1).force/1000.0f),
                                                                   16,70,16,Color{150,160,175,255});
        if(!running) DrawText("PAUSED (P)", 16,94,16,Color{240,200,90,255});

        DrawText("W/S forward/back   A/D turn   R reset   P pause",
                 16, GetScreenHeight()-26, 16, Color{120,128,140,255});

        EndDrawing();
    }

    UnloadShader(litShader);
    UnloadModel(terrain);
    CloseWindow();
    return 0;
}
