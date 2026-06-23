// =============================================================================
//  Car drivetrain simulator -- visualization / UI front-end.
//  The physics model lives in sim.h / sim.cpp; this file only renders it and
//  hosts the drivetrain editor (wheels, differentials, engine curve, clutch).
//
//  Editing is only allowed while the simulation is STOPPED.
//
//  Visualization: raylib (gauges, 3D car, plots) + raygui (sliders / editor).
//  Build with the accompanying CMakeLists.txt (FetchContent pulls the deps).
// =============================================================================
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "sim.h"
#include "physics.h"

#include <cmath>
#include <vector>
#include <algorithm>

// ----------------------------- drawing helpers ------------------------------
static void DrawGauge(Vector2 c,float R,double val,double maxv,
                      const char* label,const char* readout,
                      double redFrac=1.0){
    DrawCircle((int)c.x,(int)c.y,R+6,Color{30,33,40,255});
    DrawCircleLines((int)c.x,(int)c.y,R+6,Color{70,75,85,255});
    int N=48;
    for(int i=0;i<=N;i++){
        double f=(double)i/N;
        double a=(135 - f*270.0)*DEG2RAD;     // sweep 135deg .. -135deg
        float r0=R-(i%4==0?12:6), r1=R;
        Color col = (f>=redFrac)?Color{220,70,60,255}:Color{150,160,175,255};
        DrawLineEx({(float)(c.x+std::cos(a)*r0),(float)(c.y-std::sin(a)*r0)},
                   {(float)(c.x+std::cos(a)*r1),(float)(c.y-std::sin(a)*r1)},2,col);
    }
    double f = clampd(val/maxv,0,1);
    double a=(135 - f*270.0)*DEG2RAD;
    DrawLineEx(c,{(float)(c.x+std::cos(a)*(R-16)),(float)(c.y-std::sin(a)*(R-16))},
               4,Color{240,200,90,255});
    DrawCircle((int)c.x,(int)c.y,5,Color{240,200,90,255});
    int lw=MeasureText(label,16);
    DrawText(label,(int)c.x-lw/2,(int)c.y+R-30,16,Color{150,160,175,255});
    int rw=MeasureText(readout,22);
    DrawText(readout,(int)c.x-rw/2,(int)c.y+18,22,RAYWHITE); // below needle pivot
}

struct Plot {
    std::vector<float> data; size_t cap=320;
    void push(float v){ data.push_back(v); if(data.size()>cap) data.erase(data.begin()); }
};
static void DrawPlot(Rectangle r,const Plot& p,float lo,float hi,Color col,
                     const char* label,float zeroLine=NAN){
    DrawRectangleRec(r,Color{24,26,32,255});
    DrawRectangleLinesEx(r,1,Color{60,64,74,255});
    auto Y=[&](float v){ float t=(v-lo)/(hi-lo); t=clampd(t,0,1);
                         return r.y+r.height-t*r.height; };
    if(!std::isnan(zeroLine)){
        float yz=Y(zeroLine);
        DrawLine((int)r.x,(int)yz,(int)(r.x+r.width),(int)yz,Color{70,74,84,255});
    }
    for(size_t i=1;i<p.data.size();++i){
        float x0=r.x+(float)(i-1)/p.cap*r.width;
        float x1=r.x+(float)(i  )/p.cap*r.width;
        DrawLineEx({x0,Y(p.data[i-1])},{x1,Y(p.data[i])},2,col);
    }
    DrawText(label,(int)r.x+6,(int)r.y+4,14,col);
}

// ----------------------------- terrain --------------------------------------
// analytic heightmap in WORLD metres so the bump density stays constant no
// matter how big the map is; flattened near the spawn origin.
static float terrainH(float wx,float wz){
    float h = std::sin(wx*0.18f)*std::cos(wz*0.15f)*2.4f     // big rolling hills
            + std::sin((wx+wz)*0.11f)*1.2f
            + std::sin(wx*0.45f)*std::cos(wz*0.40f)*0.8f       // medium bumps
            + std::cos(wx*0.75f+wz*0.62f)*0.35f;               // fine chatter
    float d = std::sqrt(wx*wx+wz*wz);
    float flat = (float)clampd(1.0-d/16.0,0.0,1.0);    // ~16 m flat spawn pad
    return h*(1.0f-flat);
}

// low-frequency noise carves the map into grip zones: 0=DRY 1=WET 2=ICE
static int surfZone(float wx,float wz){
    if(wx*wx+wz*wz < 16.0f*16.0f) return 0;            // dry spawn area
    float v = std::sin(wx*0.045f+1.7f)*std::cos(wz*0.039f-0.6f)
            + 0.6f*std::sin(wx*0.021f-wz*0.027f+3.1f)
            + 0.4f*std::cos(wx*0.063f+wz*0.017f);
    if(v >  0.85f) return 1;                           // wet patch
    if(v < -0.85f) return 2;                           // icy patch
    return 0;                                          // dry
}
static Color zoneColor(int z){
    switch(z){ case 1: return Color{ 32, 74,124,255};  // WET  (deep blue)
               case 2: return Color{170,198,224,255};  // ICE  (pale blue)
               default:return Color{ 46, 56, 50,255}; }// DRY  (muted green)
}

// build a raylib mesh from the same height samples Jolt uses (exact match),
// colouring each vertex by its grip zone
static Model BuildTerrainModel(int N,float cell,const std::vector<float>& h){
    Mesh m{};
    m.vertexCount   = N*N;
    m.triangleCount = (N-1)*(N-1)*2;
    m.vertices = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.normals  = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.colors   = (unsigned char*)MemAlloc(m.vertexCount*4*sizeof(unsigned char));
    m.indices  = (unsigned short*)MemAlloc(m.triangleCount*3*sizeof(unsigned short));
    float span=(N-1)*cell;
    for(int iz=0;iz<N;iz++) for(int ix=0;ix<N;ix++){
        int v=iz*N+ix;
        float wx=-span*0.5f+cell*ix, wz=-span*0.5f+cell*iz;
        m.vertices[v*3+0]=wx;
        m.vertices[v*3+1]=h[(size_t)iz*N+ix];
        m.vertices[v*3+2]=wz;
        m.normals[v*3+0]=0; m.normals[v*3+1]=1; m.normals[v*3+2]=0;
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

// ----------------------------- 3D scene -------------------------------------
// Renders the heightmap + the Jolt rigid-body car (free to pitch/roll/yaw) with
// a smoothed chase camera, into an off-screen texture sized to `view`.
static Vector3 gCamPos = {6.0f,5.0f,8.0f};
static void DrawScene3D(Rectangle view, phys::World& world, const Vehicle& car,
                        Model terrain, int monWheel, bool spin){
    static RenderTexture2D rt{};
    if(rt.id==0) rt = LoadRenderTexture((int)view.width,(int)view.height);

    float bp[3]; world.bodyPosition(bp);
    float bq[4]; world.bodyQuat(bq);
    Quaternion q={bq[0],bq[1],bq[2],bq[3]};
    Vector3 pos={bp[0],bp[1],bp[2]};
    Vector3 fwd  = Vector3RotateByQuaternion({1,0,0},q);
    Vector3 up   = Vector3RotateByQuaternion({0,1,0},q);
    Vector3 right= Vector3RotateByQuaternion({0,0,1},q);

    // chase camera: behind + above the car, smoothed
    Vector3 want = Vector3Add(pos, Vector3Add(Vector3Scale(fwd,-7.5f),
                                              (Vector3){0,3.6f,0}));
    gCamPos = Vector3Lerp(gCamPos, want, 0.12f);
    Camera3D cam{};
    cam.position=gCamPos; cam.target=Vector3Add(pos,(Vector3){0,0.5f,0});
    cam.up={0,1,0}; cam.fovy=50.0f; cam.projection=CAMERA_PERSPECTIVE;

    BeginTextureMode(rt);
    ClearBackground(Color{20,24,34,255});
    BeginMode3D(cam);

    // terrain: zone-coloured fill (vertex colours) + faint wire grid overlay
    DrawModel(terrain,{0,0,0},1.0f,WHITE);
    DrawModelWires(terrain,{0,0,0},1.0f,Color{255,255,255,40});

    // chassis: wireframe box following the body's full orientation
    float dims[3]; world.bodyDims(dims);
    rlPushMatrix();
    rlTranslatef(pos.x,pos.y,pos.z);
    Vector3 axis; float ang; QuaternionToAxisAngle(q,&axis,&ang);
    if(Vector3Length(axis)>0.001f) rlRotatef(ang*RAD2DEG,axis.x,axis.y,axis.z);
    DrawCubeWires({0,0,0},dims[0],dims[1],dims[2],Color{120,150,200,255});
    rlPopMatrix();

    // wheels: positions come from the raycast suspension
    const auto& wo = world.wheels();
    for(size_t i=0;i<wo.size();++i){
        Vector3 c={wo[i].x,wo[i].y,wo[i].z};
        float wr=(i<car.wheels.size())?(float)car.wheels[i].r:0.42f;
        float hw=0.16f;
        Vector3 a=Vector3Subtract(c,Vector3Scale(right,hw));
        Vector3 b=Vector3Add(c,Vector3Scale(right,hw));
        bool drv=(i<car.wheels.size())&&car.wheels[i].driven;
        bool sp = drv && spin;
        Color tc= sp?Color{200,60,55,255}:(drv?Color{40,44,54,255}:Color{28,30,36,255});
        DrawCylinderEx(a,b,wr,wr,14,tc);
        DrawCylinderWiresEx(a,b,wr,wr,14,
            (int)i==monWheel?Color{150,200,240,255}:Color{110,114,124,255});
        if(!wo[i].grounded)   // mark airborne wheels
            DrawSphere(c,0.05f,Color{240,160,90,255});
        // spin spoke lies in the wheel plane (spanned by fwd & up)
        float aa=(i<car.wheels.size())?(float)car.wheels[i].angle:0.0f;
        Vector3 spoke=Vector3Add(Vector3Scale(fwd,std::cos(aa)*wr*0.9f),
                                 Vector3Scale(up,std::sin(aa)*wr*0.9f));
        DrawLine3D(c,Vector3Add(c,spoke),Color{210,214,224,255});
    }

    EndMode3D();
    EndTextureMode();

    Rectangle src={0,0,(float)rt.texture.width,-(float)rt.texture.height};
    DrawTextureRec(rt.texture, src, {view.x,view.y}, WHITE);
    DrawRectangleLinesEx(view,1,Color{60,64,74,255});
}

// ----------------------------- editor (stopped only) ------------------------
// Small reusable controls --------------------------------------------------
static void dSlider(Rectangle r,const char* l,double& val,float lo,float hi,
                    const char* fmt="%.2f"){
    float f=(float)val;
    GuiSlider(r,l,TextFormat(fmt,f),&f,lo,hi);
    val=f;
}
// "< label >" integer cycler; returns the (clamped, wrapped) value
static int cycler(float x,float y,const char* label,int val,int lo,int hi){
    if(GuiButton({x,y,28,26},"<") && val>lo) val--;
    DrawText(label,(int)x+38,(int)y+5,16,RAYWHITE);
    if(GuiButton({x+150,y,28,26},">") && val<hi) val++;
    return clampd(val,lo,hi);
}

static void DrawEditor(Rectangle panel, Vehicle& car, int& tab,
                       int& wheelSel, int& diffSel, int& pointSel, int& gearSel,
                       phys::Susp& susp, int& suspPreset){
    DrawRectangleRec(panel,Color{26,28,34,255});
    DrawRectangleLinesEx(panel,1,Color{60,64,74,255});
    DrawText("DRIVETRAIN EDITOR  (sim stopped)",(int)panel.x+12,(int)panel.y+8,16,
             Color{120,220,140,255});

    GuiToggleGroup({panel.x+12,panel.y+30,84,26},
                   "WHEELS;DIFFS;ENGINE;GEARS;CLUTCH;SUSP",&tab);

    float cx=panel.x+16, cy=panel.y+70;

    // ----------------------------- WHEELS ----------------------------------
    if(tab==0){
        int N=(int)car.wheels.size();
        wheelSel=clampd(wheelSel,0,std::max(0,N-1));
        wheelSel=cycler(cx,cy,TextFormat("Wheel %d / %d",wheelSel+1,N),wheelSel,0,N-1);

        if(GuiButton({cx+200,cy,90,26},"Add"))
            { car.addWheel(0.0,0.0,0.31,false); wheelSel=(int)car.wheels.size()-1; }
        if(GuiButton({cx+296,cy,90,26},"Remove") && N>1)
            { car.removeWheel(wheelSel); wheelSel=std::min(wheelSel,(int)car.wheels.size()-1); }

        if(N>0){
            Wheel& wh=car.wheels[wheelSel];
            dSlider({cx+70,cy+44,200,20},"Long X",wh.px,-2.5f,2.5f);
            dSlider({cx+70,cy+72,200,20},"Lat Z", wh.pz,-1.4f,1.4f);
            dSlider({cx+70,cy+100,200,20},"Radius",wh.r, 0.24f,0.55f);
            bool drv=wh.driven;
            GuiCheckBox({cx+320,cy+46,20,20}," Driven (powered)",&drv);
            wh.driven=drv;
            DrawText(TextFormat("load Fz %.0f N",wh.Fz),(int)cx+320,(int)cy+80,14,
                     Color{150,160,175,255});
        }
        DrawText("Position the wheels; tick 'Driven' to feed engine torque to them.",
                 (int)cx,(int)cy+132,13,Color{120,128,140,255});
    }
    // ----------------------------- DIFFS -----------------------------------
    else if(tab==1){
        int M=(int)car.diffs.size();
        int N=(int)car.wheels.size();
        if(M>0) diffSel=clampd(diffSel,0,M-1);
        diffSel=cycler(cx,cy,TextFormat("Diff %d / %d",M?diffSel+1:0,M),diffSel,0,std::max(0,M-1));

        if(GuiButton({cx+200,cy,90,26},"Add"))
            { car.addDiff(); diffSel=(int)car.diffs.size()-1; }
        if(GuiButton({cx+296,cy,90,26},"Remove") && M>0)
            { car.removeDiff(diffSel); diffSel=std::max(0,diffSel-1); }

        if(M>0 && N>0){
            Differential& d=car.diffs[diffSel];
            d.a=clampd(d.a,0,N-1); d.b=clampd(d.b,0,N-1);
            d.a=cycler(cx,cy+42,TextFormat("Wheel A: %d",d.a+1),d.a,0,N-1);
            d.b=cycler(cx+210,cy+42,TextFormat("Wheel B: %d",d.b+1),d.b,0,N-1);

            DrawText("Mode",(int)cx,(int)cy+78,16,Color{150,160,175,255});
            int mode=(int)d.mode;
            GuiToggleGroup({cx+60,cy+74,90,26},"OPEN;LSD;LOCKED",&mode);
            d.mode=(DiffMode)mode;

            if(d.mode!=DIFF_OPEN)
                dSlider({cx+80,cy+108,220,20},"Lock Nm",d.lockCap,50.0f,1200.0f,"%.0f");
            else
                DrawText("OPEN: equal torque, wheels free to spin independently.",
                         (int)cx,(int)cy+108,13,Color{120,128,140,255});
        } else {
            DrawText("No differentials. 'Add' to couple two wheels.",
                     (int)cx,(int)cy+44,14,Color{120,128,140,255});
        }
    }
    // ----------------------------- ENGINE ----------------------------------
    else if(tab==2){
        int P=(int)car.eng.curve.size();
        if(P>0) pointSel=clampd(pointSel,0,P-1);
        pointSel=cycler(cx,cy,TextFormat("Point %d / %d",P?pointSel+1:0,P),pointSel,0,std::max(0,P-1));

        if(GuiButton({cx+200,cy,90,26},"Add")){
            double r = P>0 ? car.eng.curve.back().rpm+500.0 : 1000.0;
            car.eng.curve.push_back({r,200.0}); car.eng.sortCurve();
            pointSel=(int)car.eng.curve.size()-1;
        }
        if(GuiButton({cx+296,cy,90,26},"Remove") && P>1)
            { car.eng.curve.erase(car.eng.curve.begin()+pointSel);
              pointSel=std::max(0,pointSel-1); }

        if(P>0){
            CurvePoint& cp=car.eng.curve[pointSel];
            dSlider({cx+70,cy+42,210,20},"RPM",cp.rpm,0.0f,7500.0f,"%.0f");
            dSlider({cx+70,cy+70,210,20},"Nm", cp.nm, 0.0f,400.0f,"%.0f");
            car.eng.sortCurve();
            dSlider({cx+70,cy+100,210,20},"Redline",car.eng.redline,3000.0f,7500.0f,"%.0f");
        }

        // mini torque-curve preview on the right of the panel
        Rectangle gr={panel.x+440,panel.y+64,panel.width-456,panel.height-78};
        DrawRectangleRec(gr,Color{20,22,28,255});
        DrawRectangleLinesEx(gr,1,Color{60,64,74,255});
        auto GX=[&](double rpm){ return gr.x + (float)(rpm/7500.0)*gr.width; };
        auto GY=[&](double nm){ return gr.y+gr.height-(float)(nm/400.0)*gr.height; };
        for(size_t i=1;i<car.eng.curve.size();++i)
            DrawLineEx({GX(car.eng.curve[i-1].rpm),GY(car.eng.curve[i-1].nm)},
                       {GX(car.eng.curve[i].rpm),GY(car.eng.curve[i].nm)},2,
                       Color{240,200,90,255});
        for(int i=0;i<P;++i){
            Color pc = (i==pointSel)? Color{120,220,140,255}:Color{240,200,90,255};
            DrawCircle((int)GX(car.eng.curve[i].rpm),(int)GY(car.eng.curve[i].nm),4,pc);
        }
        float rx=GX(car.eng.redline);
        DrawLine((int)rx,(int)gr.y,(int)rx,(int)(gr.y+gr.height),Color{220,70,60,160});
        DrawText("torque curve (Nm vs rpm)",(int)gr.x+6,(int)gr.y+4,12,
                 Color{120,128,140,255});
    }
    // ----------------------------- GEARS -----------------------------------
    else if(tab==3){
        Gearbox& b=car.box;
        int G=b.gears();
        gearSel=clampd(gearSel,1,std::max(1,G));   // 1-based forward gear index

        gearSel=cycler(cx,cy,TextFormat("Gear %d / %d",gearSel,G),gearSel,1,G);
        if(GuiButton({cx+200,cy,90,26},"Add Gear") && G<8){
            double last = b.ratio.back();
            b.ratio.push_back(std::max(0.30,last*0.72));   // taller next gear
            gearSel=b.gears();
        }
        if(GuiButton({cx+296,cy,96,26},"Remove Gear") && G>1){
            b.ratio.pop_back();
            if(b.gear>b.gears()) b.gear=b.gears();
            gearSel=std::min(gearSel,b.gears());
        }

        if(G>=1){
            dSlider({cx+90,cy+44,220,20},
                    TextFormat("Gear %d ratio",gearSel),b.ratio[gearSel],0.30f,5.00f);
            dSlider({cx+90,cy+72,220,20},"Final drive",b.finalDrive,1.50f,6.00f);
            dSlider({cx+90,cy+100,220,20},"Efficiency",b.eff,0.70f,1.00f);
        }

        // list every gear's total reduction (ratio x final drive)
        int lx=(int)panel.x+440, ly=(int)panel.y+64;
        DrawText("ratio  x final = reduction",lx,ly,13,Color{120,128,140,255});
        for(int g=1;g<=G;g++){
            Color gc = (g==gearSel)? Color{120,220,140,255}:RAYWHITE;
            DrawText(TextFormat("%d:  %.2f  x %.2f = %.2f",g,b.ratio[g],b.finalDrive,
                     b.ratio[g]*b.finalDrive),lx,ly+18*g,14,gc);
        }
    }
    // ----------------------------- CLUTCH ----------------------------------
    else if(tab==4){
        Clutch& c=car.clu;
        DrawText("Clutch plates (count)",(int)cx,(int)cy+4,16,Color{150,160,175,255});
        if(GuiButton({cx+220,cy,30,26},"-") && c.plates>1) c.plates--;
        DrawText(TextFormat("%d",c.plates),(int)cx+262,(int)cy+4,18,RAYWHITE);
        if(GuiButton({cx+290,cy,30,26},"+") && c.plates<10) c.plates++;

        dSlider({cx+130,cy+44,200,20},"Per-plate Nm",c.capacityPerPlate,80.0f,400.0f,"%.0f");
        dSlider({cx+130,cy+74,200,20},"Lock band",c.band,2.0f,16.0f);
        DrawText(TextFormat("Total clutch capacity: %.0f Nm  (%d x %.0f)",
                 c.capacity(),c.plates,c.capacityPerPlate),
                 (int)cx,(int)cy+108,15,Color{120,200,240,255});
        DrawText("More plates -> more torque before the clutch slips.",
                 (int)cx,(int)cy+132,13,Color{120,128,140,255});
    }
    // ----------------------------- SUSPENSION ------------------------------
    else {
        DrawText("Preset",(int)cx,(int)cy+4,16,Color{150,160,175,255});
        int old=suspPreset;
        GuiToggleGroup({cx+70,cy,90,26},"SPORT;COMFORT;OFFROAD",&suspPreset);
        if(suspPreset!=old) susp=phys::suspPreset(suspPreset);  // apply preset

        auto fslider=[&](Rectangle r,const char*l,float&v,float lo,float hi,
                         const char*fmt){
            GuiSlider(r,l,TextFormat(fmt,v),&v,lo,hi); };
        fslider({cx+110,cy+40,200,18},"Rest len",  susp.rest,     0.20f,0.70f,"%.2f m");
        fslider({cx+110,cy+62,200,18},"Travel",    susp.travel,   0.08f,0.45f,"%.2f m");
        fslider({cx+110,cy+84,200,18},"Stiffness", susp.stiffness,8000.f,60000.f,"%.0f");
        fslider({cx+110,cy+106,200,18},"Damping",  susp.damping,  1000.f,9000.f,"%.0f");
        fslider({cx+110,cy+128,200,18},"Wheel R",  susp.radius,   0.25f,0.55f,"%.2f m");

        int lx=(int)panel.x+440, ly=(int)panel.y+64;
        DrawText("Raycast suspension (spring + damper)",lx,ly,13,Color{120,128,140,255});
        DrawText(TextFormat("preset: %s",phys::suspPresetName(suspPreset)),
                 lx,ly+22,15,Color{120,220,140,255});
        DrawText("Each wheel casts a ray down; the spring",lx,ly+50,13,Color{120,128,140,255});
        DrawText("force holds the chassis off the terrain.",lx,ly+66,13,Color{120,128,140,255});
        DrawText("Changes apply when you press PLAY.",lx,ly+92,13,Color{200,180,120,255});
    }
}

// ----------------------------- main -----------------------------------------
int main(){
    const int W=1180,H=720;
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(W,H,"Drivetrain Simulator  -  configurable engine | clutch | diffs | wheels");
    SetTargetFPS(60);
    GuiSetStyle(DEFAULT,TEXT_SIZE,16);

    // --- engine sound: looped idle sample, pitch-shifted by rpm -------------
    InitAudioDevice();
    // look next to the executable first (CMake copies the asset there)
    Music engSnd = LoadMusicStream(TextFormat("%sidle.ogg",GetApplicationDirectory()));
    if(engSnd.frameCount==0) engSnd = LoadMusicStream("idle.ogg"); // fallback: cwd
    bool haveSnd = engSnd.frameCount>0;
    if(haveSnd){ engSnd.looping=true; PlayMusicStream(engSnd); }

    Vehicle car;
    car.external = true;                       // Jolt owns the chassis motion
    float throttle=0, clutchEng=1.0f, brake=0, steer=0;
    bool  running=true;                        // play/stop state
    bool  prevRunning=true;
    bool  soundOn=false;                       // engine audio on/off
    int   editTab=0, wheelSel=0, diffSel=0, pointSel=0, gearSel=1;
    Plot  pRPM,pWheel,pSlip,pForce;

    // --- 3D physics world: heightmap terrain + raycast-suspension vehicle ---
    const int   TN=128;           // heightfield resolution (bigger map)
    const float TCELL=2.2f;       // metres between samples -> ~280 m square
    const float TSPAN=(TN-1)*TCELL;
    std::vector<float> heights((size_t)TN*TN);
    for(int iz=0;iz<TN;iz++) for(int ix=0;ix<TN;ix++){
        float wx=-TSPAN*0.5f+TCELL*ix, wz=-TSPAN*0.5f+TCELL*iz;
        heights[(size_t)iz*TN+ix]=terrainH(wx,wz);
    }
    float spawnH = heights[(size_t)(TN/2)*TN + (TN/2)] + 1.6f;

    phys::World world;
    world.setHeightfield(TN,TCELL,heights);
    Model terrain = BuildTerrainModel(TN,TCELL,heights);

    int        suspPreset = phys::SUSP_SPORT;
    phys::Susp susp = phys::suspPreset(suspPreset);

    // (re)build the rigid-body car from the current wheel configuration
    auto buildVeh=[&](){
        if(car.wheels.empty()) return;
        std::vector<float> ox,oy,oz;
        for(const Wheel& wh: car.wheels){
            ox.push_back((float)wh.px); oy.push_back(0.0f); oz.push_back((float)wh.pz); }
        float minx=1e9f,maxx=-1e9f,minz=1e9f,maxz=-1e9f;
        for(const Wheel& wh: car.wheels){
            minx=std::min(minx,(float)wh.px); maxx=std::max(maxx,(float)wh.px);
            minz=std::min(minz,(float)wh.pz); maxz=std::max(maxz,(float)wh.pz); }
        float blen=std::max((maxx-minx)+0.6f,1.2f);
        float bwid=std::max((maxz-minz)+0.2f,0.8f);
        world.setSusp(susp);
        world.buildVehicle(ox,oy,oz,blen,0.7f,bwid,(float)car.mass,0.0f,spawnH,0.0f);
        car.reset();
    };
    buildVeh();

    while(!WindowShouldClose()){
        // ---- input (keyboard adds to the sliders) -------------------------
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))   throttle=std::min(1.0f,throttle+0.04f);
        else                                      throttle=std::max(0.0f,throttle-0.06f);
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) brake=std::min(1.0f,brake+0.06f);
        else                                      brake=std::max(0.0f,brake-0.08f);
        if(IsKeyDown(KEY_SPACE))                  clutchEng=std::max(0.0f,clutchEng-0.08f);
        else                                      clutchEng=std::min(1.0f,clutchEng+0.05f);
        float steerTgt=0.0f;                       // A/D (or arrows) steer
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))  steerTgt=-0.50f;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) steerTgt=+0.50f;
        steer += (steerTgt-steer)*0.20f;
        if(IsKeyPressed(KEY_E)&&car.box.gear<car.box.gears()) car.box.gear++;
        if(IsKeyPressed(KEY_Q)&&car.box.gear>0) car.box.gear--;
        if(IsKeyPressed(KEY_ONE))   car.surf=0;
        if(IsKeyPressed(KEY_TWO))   car.surf=1;
        if(IsKeyPressed(KEY_THREE)) car.surf=2;
        if(IsKeyPressed(KEY_P))     running=!running;          // toggle play/stop
        if(IsKeyPressed(KEY_M))     soundOn=!soundOn;          // mute/unmute sound
        if(IsKeyPressed(KEY_R)) { car.reset(); world.resetVehicle(0,spawnH,0);
                                  throttle=brake=0; clutchEng=1; }

        car.clu.engagement = clutchEng;
        world.setSusp(susp);

        // rebuild the rigid-body car when (re)starting after edits
        if(running && !prevRunning) buildVeh();
        prevRunning = running;

        // ---- physics: drivetrain + 3D rigid body (only while running) -----
        if(running && world.hasVehicle()){
            // grip follows the terrain zone the car is currently sitting on
            float bp0[3]; world.bodyPosition(bp0);
            car.surf = surfZone(bp0[0], bp0[2]);

            // feed the body state back into the drivetrain
            car.v = world.forwardSpeed();
            const auto& wo=world.wheels();
            for(size_t i=0;i<car.wheels.size() && i<wo.size();++i)
                car.wheels[i].Fz = wo[i].grounded ? wo[i].Fz : 0.0;

            // advance the drivetrain (engine/clutch/gearbox/wheel spin -> Fx)
            const int SUB=8; double dt=(1.0/60.0)/SUB;
            for(int s=0;s<SUB;s++) car.step(dt,throttle,brake);

            // hand each wheel's longitudinal tire force to Jolt and step it
            std::vector<float> driveN(car.wheels.size());
            for(size_t i=0;i<car.wheels.size();++i) driveN[i]=(float)car.wheels[i].Fx;
            world.step(1.0f/60.0f, driveN, brake, steer, SURFACES[car.surf].mu);

            // fell off the terrain? respawn at the centre pad
            float bp[3]; world.bodyPosition(bp);
            float half = TSPAN*0.5f;
            if(bp[1] < -10.0f || std::fabs(bp[0])>half+4.0f || std::fabs(bp[2])>half+4.0f){
                world.resetVehicle(0,spawnH,0); car.reset();
            }
        }

        // ---- telemetry (pick a representative driven wheel to monitor) ----
        int mon=-1;
        for(int i=0;i<(int)car.wheels.size();++i) if(car.wheels[i].driven){ mon=i; break; }
        if(mon<0) mon = car.wheels.empty()? 0 : 0;
        bool haveW = !car.wheels.empty();

        double rpm  = car.eng.omega*RAD2RPM;
        double kmh  = car.v*3.6;
        double n    = car.box.n();
        double carrier=0.0; int nd=car.drivenCount();
        if(nd>0){ for(const Wheel& wh:car.wheels) if(wh.driven) carrier+=wh.omega; carrier/=nd; }
        double wheelRPM = carrier*n*RAD2RPM;                  // engine-equiv
        double monFz = haveW? car.wheels[mon].Fz : 0.0;
        double monK  = haveW? car.wheels[mon].kappa : 0.0;
        double monFx = haveW? car.wheels[mon].Fx : 0.0;
        double kP   = car.tire.kappaPeak(SURFACES[car.surf].mu, std::max(monFz,1.0));
        if(running){
            pRPM.push((float)rpm); pWheel.push((float)wheelRPM);
            pSlip.push((float)monK); pForce.push((float)monFx);
        }
        bool spin = std::fabs(monK) > kP*1.05 && throttle>0.05;

        // ---- engine audio: pitch tracks rpm, muted when stalled/stopped ----
        if(haveSnd){
            // map the whole idle->redline span across the pitch range so the
            // full rev range is audible (not saturated near the bottom)
            double frac = (rpm - car.eng.idleRPM)/(car.eng.redline - car.eng.idleRPM);
            frac = clampd(frac, 0.0, 1.0);
            float pitch = (float)(0.85 + frac*2.15);   // ~0.85 idle .. ~3.0 redline
            SetMusicPitch(engSnd, pitch);
            bool silent = !soundOn || car.eng.stalled || !running;
            SetMusicVolume(engSnd, silent?0.0f:1.0f);
            UpdateMusicStream(engSnd);
        }

        // ===================== render ======================================
        BeginDrawing();
        ClearBackground(Color{18,20,25,255});

        // --- left: live controls (raygui) ----------------------------------
        DrawRectangle(0,0,300,H,Color{26,28,34,255});
        DrawText("PARAMETERS",20,16,18,Color{150,160,175,255});
        GuiSlider({110,52,150,22},"Throttle",TextFormat("%.2f",throttle),&throttle,0,1);
        GuiSlider({110,82,150,22},"Clutch",  TextFormat("%.2f",clutchEng),&clutchEng,0,1);
        GuiSlider({110,112,150,22},"Brake",  TextFormat("%.2f",brake),&brake,0,1);

        DrawText("GEAR",20,150,16,Color{150,160,175,255});
        if(GuiButton({90,146,40,26},"-")&&car.box.gear>0) car.box.gear--;
        const char* gname = car.box.gear==0?"N":TextFormat("%d",car.box.gear);
        DrawText(gname,150,150,20,RAYWHITE);
        if(GuiButton({180,146,40,26},"+")&&car.box.gear<car.box.gears()) car.box.gear++;

        DrawText("SURFACE",20,190,16,Color{150,160,175,255});
        int surfSel=car.surf;
        GuiToggleGroup({100,186,55,26},"DRY;WET;ICE",&surfSel);
        car.surf=surfSel;

        // --- play / stop simulation toggle ---------------------------------
        Color simBtnCol = running ? Color{200,80,70,255} : Color{90,180,110,255};
        int prevBase  = GuiGetStyle(BUTTON,BASE_COLOR_NORMAL);
        GuiSetStyle(BUTTON,BASE_COLOR_NORMAL,ColorToInt(simBtnCol));
        if(GuiButton({20,228,170,30}, running?"#132#STOP  (P)":"#131#PLAY  (P)"))
            running=!running;
        GuiSetStyle(BUTTON,BASE_COLOR_NORMAL,prevBase);
        if(GuiButton({200,228,80,30},"Reset")){ car.reset(); throttle=brake=0; clutchEng=1; }
        DrawText(running?"SIM: RUNNING":"SIM: STOPPED (editable)",20,266,14,
                 running?Color{120,220,140,255}:Color{240,200,90,255});

        if(GuiButton({200,262,80,26}, soundOn?"#122#Sound":"#123#Muted")) soundOn=!soundOn;

        DrawText("KEYS: W throttle  S brake  A/D steer",20,294,14,Color{120,128,140,255});
        DrawText("SPACE clutch  Q/E shift",20,314,14,Color{120,128,140,255});
        DrawText("1/2/3 surface  P play  M mute  R reset",20,334,14,Color{120,128,140,255});

        // readouts
        int ry=368;
        auto line=[&](const char*k,const char*v,Color c){
            DrawText(k,20,ry,16,Color{150,160,175,255});
            DrawText(v,150,ry,16,c); ry+=24; };
        line("Engine", car.eng.stalled?"STALLED":TextFormat("%.0f rpm",rpm),
                       car.eng.stalled?Color{240,90,80,255}:RAYWHITE);
        line("Speed",  TextFormat("%.1f km/h",kmh),RAYWHITE);
        line("Clutch", car.clu.lockedish?"LOCKED":"slipping",
                       car.clu.lockedish?Color{120,220,140,255}:Color{240,200,90,255});
        line("Drive",  TextFormat("%d driven / %d wheels",nd,(int)car.wheels.size()),RAYWHITE);
        line("Diffs",  TextFormat("%d",(int)car.diffs.size()),RAYWHITE);
        line("k_peak", TextFormat("%.3f",kP), Color{150,200,240,255});
        line("mon k",  TextFormat("%.3f",monK), spin?Color{240,90,80,255}:RAYWHITE);
        line("mon Fx", TextFormat("%.0f N",monFx),RAYWHITE);
        if(spin) DrawText("WHEELSPIN",150,ry,18,Color{240,90,80,255});
        if(car.eng.stalled)
            DrawText("ENGINE STALLED\n-> clutch in / neutral to restart",20,ry+22,14,
                     Color{240,160,90,255});

        // --- gauges --------------------------------------------------------
        DrawGauge({470,150},110,rpm,7000,"RPM x1000",
                  TextFormat("%.1f",rpm/1000.0), car.eng.redline/7000.0);
        DrawGauge({470+260,150},110,std::fabs(kmh),220,"km/h",
                  TextFormat("%.0f",std::fabs(kmh)),1.0);

        // --- 3D scene: heightmap + rigid-body car --------------------------
        Rectangle scene={330,278,820,170};
        DrawScene3D(scene, world, car, terrain, mon, spin);
        DrawText(TextFormat("gear %s   %s   %d wheels   A/D steer", gname,
                 SURFACES[car.surf].name,(int)car.wheels.size()),
                 (int)scene.x+10,(int)scene.y+8,16,Color{180,188,200,255});

        // --- lower area: plots while running, editor while stopped ---------
        Rectangle lower={330,458,820,252};
        if(running){
            float py=lower.y, pw=400, ph=116;
            DrawPlot({lower.x,py,pw,ph},pRPM,0,7000,Color{240,200,90,255},"engine rpm");
            DrawPlot({lower.x,py,pw,ph},pWheel,0,7000,Color{120,200,240,255},
                     "wheel-equiv rpm",NAN);
            DrawText("engine vs wheel-equiv rpm (gap = clutch slip)",
                     (int)lower.x+6,(int)(py+ph-18),12,Color{120,128,140,255});

            DrawPlot({lower.x+pw+20,py,pw,ph},pSlip,-0.5f,0.5f,Color{240,120,90,255},
                     "monitored slip k",0.0f);
            {
                Rectangle r={lower.x+pw+20,py,pw,ph};
                auto Yk=[&](float v){ float t=(v+0.5f)/1.0f; t=clampd(t,0,1);
                                      return r.y+r.height-t*r.height; };
                float yp=Yk((float)kP), ym=Yk(-(float)kP);
                DrawLine((int)r.x,(int)yp,(int)(r.x+r.width),(int)yp,Color{90,160,90,180});
                DrawLine((int)r.x,(int)ym,(int)(r.x+r.width),(int)ym,Color{90,160,90,180});
                DrawText("green = +/-k_peak (past it = sliding)",
                         (int)r.x+6,(int)(r.y+r.height-18),12,Color{120,128,140,255});
            }
            DrawPlot({lower.x,py+ph+14,pw*2+20,ph},pForce,-6000,6000,
                     Color{150,220,150,255},"monitored longitudinal force Fx [N]",0.0f);
        } else {
            DrawEditor(lower, car, editTab, wheelSel, diffSel, pointSel, gearSel,
                       susp, suspPreset);
        }

        EndDrawing();
    }
    UnloadModel(terrain);
    if(haveSnd) UnloadMusicStream(engSnd);
    CloseAudioDevice();
    CloseWindow();
    return 0;
}
