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
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "sim.h"

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

// ----------------------------- 3D car scene ---------------------------------
// Draws the configured car (body sized to the wheel layout + cylinder wheels)
// on a scrolling grid, rendered into an off-screen texture sized to `view`.
static void DrawCar3D(Rectangle view, const Vehicle& car, double groundScroll,
                      int monWheel, bool spin){
    static RenderTexture2D rt{};
    if(rt.id==0) rt = LoadRenderTexture((int)view.width,(int)view.height);

    Camera3D cam{};
    cam.position   = {4.6f, 3.0f, 5.4f};
    cam.target     = {0.0f, 0.4f, 0.0f};
    cam.up         = {0.0f, 1.0f, 0.0f};
    cam.fovy       = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    BeginTextureMode(rt);
    ClearBackground(Color{20,24,34,255});
    BeginMode3D(cam);

    // --- ground: a grid that scrolls along X (the car's forward axis) -------
    float tile = 2.0f;
    float off  = (float)std::fmod(groundScroll, (double)tile);
    rlPushMatrix();
    rlTranslatef(-off, 0.0f, 0.0f);
    DrawGrid(40, tile);
    rlPopMatrix();

    // --- body: a simple wireframe cube enclosing the wheels ----------------
    if(!car.wheels.empty()){
        float minx=1e9f,maxx=-1e9f,minz=1e9f,maxz=-1e9f;
        for(const Wheel& wh: car.wheels){
            minx=std::min(minx,(float)wh.px); maxx=std::max(maxx,(float)wh.px);
            minz=std::min(minz,(float)wh.pz); maxz=std::max(maxz,(float)wh.pz);
        }
        float cxw=(minx+maxx)*0.5f, czw=(minz+maxz)*0.5f;
        float blen=std::max((maxx-minx)+0.6f, 1.2f);
        float bwid=std::max((maxz-minz)+0.2f, 0.8f);
        DrawCubeWires({cxw, 0.6f, czw}, blen, 0.8f, bwid, Color{120,150,200,255});
    }

    // --- wheels: cylinders along Z, with a spoke to show spin ---------------
    for(int i=0;i<(int)car.wheels.size();++i){
        const Wheel& wh=car.wheels[i];
        float wr=(float)wh.r;
        float ww=0.26f;                          // half-width along the axle
        Vector3 hub = { (float)wh.px, wr, (float)wh.pz };
        Vector3 a   = { hub.x, hub.y, hub.z - ww };
        Vector3 b   = { hub.x, hub.y, hub.z + ww };
        bool sp  = wh.driven && spin;
        Color tc = sp ? Color{200,60,55,255}
                       : (wh.driven? Color{40,44,54,255} : Color{28,30,36,255});
        DrawCylinderEx(a, b, wr, wr, 16, tc);
        DrawCylinderWiresEx(a, b, wr, wr, 16,
            i==monWheel? Color{150,200,240,255} : Color{110,114,124,255});

        float ang=(float)wh.angle;
        Vector3 tip = { hub.x + std::cos(ang)*wr*0.9f,
                        hub.y + std::sin(ang)*wr*0.9f, hub.z };
        DrawLine3D({hub.x,hub.y,hub.z-ww*1.05f},{tip.x,tip.y,hub.z-ww*1.05f},
                   Color{210,214,224,255});
        DrawLine3D({hub.x,hub.y,hub.z+ww*1.05f},{tip.x,tip.y,hub.z+ww*1.05f},
                   Color{210,214,224,255});
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
                       int& wheelSel, int& diffSel, int& pointSel, int& gearSel){
    DrawRectangleRec(panel,Color{26,28,34,255});
    DrawRectangleLinesEx(panel,1,Color{60,64,74,255});
    DrawText("DRIVETRAIN EDITOR  (sim stopped)",(int)panel.x+12,(int)panel.y+8,16,
             Color{120,220,140,255});

    GuiToggleGroup({panel.x+12,panel.y+30,96,26},
                   "WHEELS;DIFFS;ENGINE;GEARS;CLUTCH",&tab);

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
    else {
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
    float throttle=0, clutchEng=1.0f, brake=0;
    bool  running=true;                       // play/stop state
    bool  soundOn=false;                        // engine audio on/off
    int   editTab=0, wheelSel=0, diffSel=0, pointSel=0, gearSel=1;
    Plot  pRPM,pWheel,pSlip,pForce;
    double groundScroll=0;

    while(!WindowShouldClose()){
        // ---- input (keyboard adds to the sliders) -------------------------
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))   throttle=std::min(1.0f,throttle+0.04f);
        else                                      throttle=std::max(0.0f,throttle-0.06f);
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) brake=std::min(1.0f,brake+0.06f);
        else                                      brake=std::max(0.0f,brake-0.08f);
        if(IsKeyDown(KEY_SPACE))                  clutchEng=std::max(0.0f,clutchEng-0.08f);
        else                                      clutchEng=std::min(1.0f,clutchEng+0.05f);
        if(IsKeyPressed(KEY_E)&&car.box.gear<car.box.gears()) car.box.gear++;
        if(IsKeyPressed(KEY_Q)&&car.box.gear>0) car.box.gear--;
        if(IsKeyPressed(KEY_ONE))   car.surf=0;
        if(IsKeyPressed(KEY_TWO))   car.surf=1;
        if(IsKeyPressed(KEY_THREE)) car.surf=2;
        if(IsKeyPressed(KEY_P))     running=!running;          // toggle play/stop
        if(IsKeyPressed(KEY_M))     soundOn=!soundOn;          // mute/unmute sound
        if(IsKeyPressed(KEY_R))     { car.reset(); throttle=brake=0; clutchEng=1; }

        car.clu.engagement = clutchEng;

        // ---- physics: substep the driveline (only while running) ----------
        if(running){
            const int SUB=8; double dt=(1.0/60.0)/SUB;
            for(int s=0;s<SUB;s++) car.step(dt,throttle,brake);
            groundScroll += car.v*GetFrameTime()*1.2;
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

        DrawText("KEYS: W throttle  S brake",20,294,14,Color{120,128,140,255});
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

        // --- 3D car scene --------------------------------------------------
        Rectangle scene={330,278,820,170};
        DrawCar3D(scene, car, groundScroll, mon, spin);
        DrawText(TextFormat("gear %s   %s   %d wheels", gname,
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
            DrawEditor(lower, car, editTab, wheelSel, diffSel, pointSel, gearSel);
        }

        EndDrawing();
    }
    if(haveSnd) UnloadMusicStream(engSnd);
    CloseAudioDevice();
    CloseWindow();
    return 0;
}
