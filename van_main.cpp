// =============================================================================
//  Ford Transit Mk7 330M -- CUSTOM DRIVETRAIN simulator.
//
//  Unlike carsim (which hands the drivetrain to Jolt's WheeledVehicleController)
//  this build runs OUR OWN drivetrain end to end:
//
//      Engine (torque curve, real stall)  ->  Clutch (soft-locking friction)
//        ->  Gearbox  ->  Final drive  ->  Differentials  ->  Pacejka tyres
//
//  Jolt is used ONLY for the rigid-body chassis, the suspension raycasts and
//  the collision.  Because the engine has no minimum-RPM clamp, it dies when
//  the load drags it under its stall speed -- pulling away in 5th, or braking
//  to a stop in gear, kills it exactly like the real van.
//
//  Per frame:
//     1. read each wheel's load + contact speed back from the raycast rig
//     2. advance the drivetrain (substepped) -> wheel spin -> tyre forces
//     3. push those tyre forces into Jolt and integrate the chassis
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
        double a=(135 - f*270.0)*DEG2RAD;
        float r0=R-(i%4==0?12:6), r1=R;
        Color col = (f>=redFrac)?Color{220,70,60,255}:Color{150,160,175,255};
        DrawLineEx({(float)(c.x+std::cos(a)*r0),(float)(c.y-std::sin(a)*r0)},
                   {(float)(c.x+std::cos(a)*r1),(float)(c.y-std::sin(a)*r1)},2,col);
    }
    double f = clampd(val/maxv,0,1);
    double a=(135 - f*270.0)*DEG2RAD;
    DrawLineEx(c,{(float)(c.x+std::cos(a)*(R-R*0.15f)),(float)(c.y-std::sin(a)*(R-R*0.15f))},
               std::max(2.0f,R*0.04f),Color{240,200,90,255});
    DrawCircle((int)c.x,(int)c.y,std::max(3.0f,R*0.05f),Color{240,200,90,255});
    int lf=std::max(9,(int)(R*0.20f));
    int rf=std::max(13,(int)(R*0.32f));
    int lw=MeasureText(label,lf);
    DrawText(label,(int)c.x-lw/2,(int)(c.y+R*0.50f),lf,Color{150,160,175,255});
    int rw=MeasureText(readout,rf);
    DrawText(readout,(int)c.x-rw/2,(int)(c.y+R*0.14f),rf,RAYWHITE);
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

static void dSlider(Rectangle r,const char* label,double& v,float lo,float hi,
                    const char* fmt="%.2f"){
    float f=(float)v;
    GuiSlider(r,label,TextFormat(fmt,f),&f,lo,hi);
    v=f;
}

// ----------------------------- terrain --------------------------------------
static bool gFlatTerrain = false;

static float terrainH(float wx,float wz){
    if(gFlatTerrain) return 0.0f;
    float h = std::sin(wx*0.18f)*std::cos(wz*0.15f)*2.4f
            + std::sin((wx+wz)*0.11f)*1.2f
            + std::sin(wx*0.45f)*std::cos(wz*0.40f)*0.8f
            + std::cos(wx*0.75f+wz*0.62f)*0.35f;
    float d = std::sqrt(wx*wx+wz*wz);
    float flat = (float)clampd((22.0-d)/8.0,0.0,1.0);
    return h*(1.0f-flat);
}

static int surfZone(float wx,float wz){
    if(wx*wx+wz*wz < 16.0f*16.0f) return 0;
    float v = std::sin(wx*0.045f+1.7f)*std::cos(wz*0.039f-0.6f)
            + 0.6f*std::sin(wx*0.021f-wz*0.027f+3.1f)
            + 0.4f*std::cos(wx*0.063f+wz*0.017f);
    if(v >  0.85f) return 1;
    if(v < -0.85f) return 2;
    return 0;
}
static Color zoneColor(int z){
    switch(z){ case 1: return Color{ 32, 74,124,255};
               case 2: return Color{170,198,224,255};
               default:return Color{ 46, 56, 50,255}; }
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
static Vector3 gCamPos = {6.0f,5.0f,8.0f};
// start behind the van (it spawns facing +X) so left on the keyboard is left on
// screen -- an orbit that starts ahead of the nose mirrors the steering
static float   gOrbitYaw=-1.9f, gOrbitPitch=0.42f, gOrbitDist=14.0f;

static void DrawScene3D(Rectangle view, phys::World& world, const Vehicle& car,
                        Model terrain, const phys::Susp& susp, Vector3 com){
    static RenderTexture2D rt{};
    if(rt.id==0 || rt.texture.width!=(int)view.width
                || rt.texture.height!=(int)view.height){
        if(rt.id!=0) UnloadRenderTexture(rt);
        rt = LoadRenderTexture((int)view.width,(int)view.height);
    }

    float bp[3]; world.bodyPosition(bp);
    float bq[4]; world.bodyQuat(bq);
    Quaternion q={bq[0],bq[1],bq[2],bq[3]};
    Vector3 pos={bp[0],bp[1],bp[2]};
    Vector3 fwd  = Vector3RotateByQuaternion({1,0,0},q);
    Vector3 up   = Vector3RotateByQuaternion({0,1,0},q);
    Vector3 right= Vector3RotateByQuaternion({0,0,1},q);

    if(CheckCollisionPointRec(GetMousePosition(),view)){
        if(IsMouseButtonDown(MOUSE_BUTTON_LEFT) ||
           IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
            Vector2 d=GetMouseDelta();
            gOrbitYaw   -= d.x*0.006f;
            gOrbitPitch += d.y*0.006f;
            gOrbitPitch  = (float)clampd(gOrbitPitch,0.08f,1.45f);
        }
        float wheel=GetMouseWheelMove();
        if(wheel!=0) gOrbitDist=(float)clampd(gOrbitDist-wheel*1.5f,4.0f,45.0f);
    }
    Vector3 tgt = Vector3Add(pos,Vector3{0,0.5f,0});
    Vector3 off = { std::cos(gOrbitPitch)*std::sin(gOrbitYaw),
                    std::sin(gOrbitPitch),
                    std::cos(gOrbitPitch)*std::cos(gOrbitYaw) };
    Vector3 want = Vector3Add(tgt, Vector3Scale(off, gOrbitDist));
    gCamPos = Vector3Lerp(gCamPos, want, 0.30f);
    Camera3D cam{};
    cam.position=gCamPos; cam.target=tgt;
    cam.up={0,1,0}; cam.fovy=50.0f; cam.projection=CAMERA_PERSPECTIVE;

    BeginTextureMode(rt);
    ClearBackground(Color{20,24,34,255});
    BeginMode3D(cam);

    DrawModel(terrain,{0,0,0},1.0f,WHITE);
    DrawModelWires(terrain,{0,0,0},1.0f,Color{255,255,255,40});

    // chassis box
    float dims[3]; world.bodyDims(dims);
    rlPushMatrix();
    rlTranslatef(pos.x,pos.y,pos.z);
    Vector3 axis; float ang; QuaternionToAxisAngle(q,&axis,&ang);
    if(Vector3Length(axis)>0.001f) rlRotatef(ang*RAD2DEG,axis.x,axis.y,axis.z);
    DrawCubeWires({0,0,0},dims[0],dims[1],dims[2],Color{120,150,200,255});
    rlPopMatrix();

    Vector3 comW=Vector3Add(pos,Vector3RotateByQuaternion({com.x,com.y,com.z},q));
    DrawSphere(comW,0.13f,Color{235,80,70,255});

    // wheels + suspension rays
    const auto& wo = world.wheels();
    for(size_t i=0;i<wo.size();++i){
        Vector3 c={wo[i].x,wo[i].y,wo[i].z};
        float wr=(i<car.wheels.size())?(float)car.wheels[i].r:0.345f;
        float hw=0.118f;
        float st=wo[i].steer;
        // must match the wheel axes stepRig() builds, or the wheels appear to
        // steer the opposite way to the direction the tyre forces push the van
        Vector3 wFwd  =Vector3Add(Vector3Scale(fwd,  std::cos(st)),
                                  Vector3Scale(right,std::sin(st)));
        Vector3 wRight=Vector3Subtract(Vector3Scale(right,std::cos(st)),
                                       Vector3Scale(fwd,  std::sin(st)));
        // suspension ray from the attach point down to the contact
        if(i<car.wheels.size()){
            Vector3 local={(float)car.wheels[i].px,
                           -dims[1]*0.5f+0.50f,
                           (float)car.wheels[i].pz};
            Vector3 attach=Vector3Add(pos,Vector3RotateByQuaternion(local,q));
            Color rc = wo[i].grounded ? Color{110,200,120,160}
                                      : Color{240,160,90,140};
            DrawLine3D(attach,c,rc);
        }

        Vector3 a=Vector3Subtract(c,Vector3Scale(wRight,hw));
        Vector3 b=Vector3Add(c,Vector3Scale(wRight,hw));
        bool drv=(i<car.wheels.size())&&car.wheels[i].driven;
        double kap=(i<car.wheels.size())?std::fabs(car.wheels[i].kappa):0.0;
        bool sp = kap>0.12;
        Color tc= sp?Color{200,60,55,255}:(drv?Color{40,44,54,255}:Color{28,30,36,255});
        DrawCylinderEx(a,b,wr,wr,14,tc);
        DrawCylinderWiresEx(a,b,wr,wr,14,Color{110,114,124,255});
        if(!wo[i].grounded) DrawSphere(c,0.05f,Color{240,160,90,255});

        float aa=(i<car.wheels.size())?(float)car.wheels[i].angle:0.0f;
        Vector3 fa=Vector3Add(a,Vector3Scale(wRight,-0.02f));
        Vector3 fb=Vector3Add(b,Vector3Scale(wRight, 0.02f));
        const int SPK=4;
        for(int s=0;s<SPK;s++){
            float angle=aa + s*(PI*2.0f/SPK);
            Vector3 dir=Vector3Add(Vector3Scale(wFwd,std::cos(angle)*wr*0.86f),
                                   Vector3Scale(up,  std::sin(angle)*wr*0.86f));
            Color sc = (s==0)?Color{245,210,120,255}:Color{210,214,224,255};
            DrawLine3D(fa,Vector3Add(fa,dir),sc);
            DrawLine3D(fb,Vector3Add(fb,dir),sc);
        }
    }

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
            Color fill={78,90,108,255}, wire={150,160,175,255};
            rlDisableBackfaceCulling();
            DrawTriangle3D(wv[0],wv[1],wv[5],fill); DrawTriangle3D(wv[0],wv[5],wv[4],fill);
            DrawTriangle3D(wv[0],wv[2],wv[3],fill); DrawTriangle3D(wv[0],wv[3],wv[1],fill);
            DrawTriangle3D(wv[2],wv[4],wv[5],fill); DrawTriangle3D(wv[2],wv[5],wv[3],fill);
            DrawTriangle3D(wv[0],wv[4],wv[2],fill); DrawTriangle3D(wv[1],wv[3],wv[5],fill);
            rlEnableBackfaceCulling();
            DrawLine3D(wv[0],wv[2],wire); DrawLine3D(wv[1],wv[3],wire);
            DrawLine3D(wv[4],wv[5],wire); DrawLine3D(wv[2],wv[4],wire);
            DrawLine3D(wv[3],wv[5],wire);
        }
    }

    EndMode3D();
    EndTextureMode();

    DrawTextureRec(rt.texture,{0,0,(float)rt.texture.width,-(float)rt.texture.height},
                   {view.x,view.y},WHITE);
    DrawRectangleLinesEx(view,1,Color{60,64,74,255});
}

// ----------------------------- tuning panel ---------------------------------
static void DrawTuning(Rectangle panel, Vehicle& car, int& tab, int& pointSel,
                       int& gearSel, phys::Susp& susp, int& suspPreset){
    DrawRectangleRec(panel,Color{26,28,34,255});
    DrawRectangleLinesEx(panel,1,Color{60,64,74,255});
    DrawText("DRIVETRAIN TUNING  (sim stopped)",(int)panel.x+12,(int)panel.y+8,16,
             Color{120,220,140,255});
    GuiToggleGroup({panel.x+12,panel.y+30,84,26},"ENGINE;GEARS;CLUTCH;BODY;SUSP",&tab);

    float cx=panel.x+16, cy=panel.y+70;

    if(tab==0){                                   // ---------------- ENGINE
        int P=(int)car.eng.curve.size();
        if(P>0) pointSel=clampd(pointSel,0,P-1);
        if(GuiButton({cx,cy,28,26},"<") && pointSel>0) pointSel--;
        DrawText(TextFormat("Point %d / %d",P?pointSel+1:0,P),(int)cx+38,(int)cy+5,16,RAYWHITE);
        if(GuiButton({cx+150,cy,28,26},">") && pointSel<P-1) pointSel++;
        if(P>0){
            CurvePoint& cp=car.eng.curve[pointSel];
            dSlider({cx+70,cy+42,210,20},"RPM",cp.rpm,0.0f,5000.0f,"%.0f");
            dSlider({cx+70,cy+70,210,20},"Nm", cp.nm, 0.0f,450.0f,"%.0f");
            car.eng.sortCurve();
            dSlider({cx+70,cy+100,210,20},"Redline",car.eng.redline,1500.0f,6000.0f,"%.0f");
            dSlider({cx+70,cy+128,210,20},"Stall rpm",car.eng.stallRPM,150.0f,900.0f,"%.0f");
        }
        Rectangle gr={panel.x+340,panel.y+64,panel.width-356,panel.height-78};
        DrawRectangleRec(gr,Color{20,22,28,255});
        DrawRectangleLinesEx(gr,1,Color{60,64,74,255});
        auto GX=[&](double rpm){ return gr.x + (float)(rpm/5000.0)*gr.width; };
        auto GY=[&](double nm){ return gr.y+gr.height-(float)(nm/450.0)*gr.height; };
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
        float sx=GX(car.eng.stallRPM);
        DrawLine((int)sx,(int)gr.y,(int)sx,(int)(gr.y+gr.height),Color{90,140,220,160});
        DrawText("torque curve (Nm vs rpm)",(int)gr.x+6,(int)gr.y+4,12,
                 Color{120,128,140,255});
    }
    else if(tab==1){                              // ---------------- GEARS
        Gearbox& b=car.box;
        int G=b.gears();
        gearSel=clampd(gearSel,1,std::max(1,G));
        if(GuiButton({cx,cy,28,26},"<") && gearSel>1) gearSel--;
        DrawText(TextFormat("Gear %d / %d",gearSel,G),(int)cx+38,(int)cy+5,16,RAYWHITE);
        if(GuiButton({cx+150,cy,28,26},">") && gearSel<G) gearSel++;
        if(GuiButton({cx+200,cy,90,26},"Add") && G<8){
            b.ratio.push_back(std::max(0.30,b.ratio.back()*0.85)); gearSel=b.gears(); }
        if(GuiButton({cx+296,cy,96,26},"Remove") && G>1){
            b.ratio.pop_back();
            if(b.gear>b.gears()) b.gear=b.gears();
            gearSel=std::min(gearSel,b.gears()); }
        if(G>=1){
            dSlider({cx+90,cy+44,220,20},
                    TextFormat("Gear %d",gearSel),b.ratio[gearSel],0.30f,5.00f);
            dSlider({cx+90,cy+72,220,20},"Final drive",b.finalDrive,1.50f,6.00f);
            dSlider({cx+90,cy+100,220,20},"Efficiency",b.eff,0.70f,1.00f);
        }
        int lx=(int)panel.x+400, ly=(int)panel.y+64;
        DrawText("ratio x final = reduction",lx,ly,13,Color{120,128,140,255});
        for(int g=1;g<=G;g++){
            Color gc = (g==gearSel)? Color{120,220,140,255}:RAYWHITE;
            DrawText(TextFormat("%d:  %.2f x %.2f = %.2f",g,b.ratio[g],b.finalDrive,
                     b.ratio[g]*b.finalDrive),lx,ly+18*g,14,gc);
        }
    }
    else if(tab==2){                              // ---------------- CLUTCH
        Clutch& c=car.clu;
        DrawText("Clutch plates",(int)cx,(int)cy+4,16,Color{150,160,175,255});
        if(GuiButton({cx+220,cy,30,26},"-") && c.plates>1) c.plates--;
        DrawText(TextFormat("%d",c.plates),(int)cx+262,(int)cy+4,18,RAYWHITE);
        if(GuiButton({cx+290,cy,30,26},"+") && c.plates<10) c.plates++;
        dSlider({cx+130,cy+44,200,20},"Per-plate Nm",c.capacityPerPlate,80.0f,700.0f,"%.0f");
        dSlider({cx+130,cy+74,200,20},"Lock band",c.band,2.0f,16.0f);
        DrawText(TextFormat("Total capacity: %.0f Nm",c.capacity()),
                 (int)cx,(int)cy+108,15,Color{120,200,240,255});
        DrawText("Capacity below peak engine torque -> it slips instead of stalling.",
                 (int)cx,(int)cy+132,13,Color{120,128,140,255});
    }
    else if(tab==3){                              // ---------------- BODY
        dSlider({cx+130,cy+4,210,20},"Vehicle mass",car.mass,600.0f,3800.0f,"%.0f kg");
        dSlider({cx+130,cy+34,210,20},"Flywheel I",car.eng.I,0.10f,0.90f,"%.2f");
        dSlider({cx+130,cy+64,210,20},"Wheel I",car.wheels.empty()?car.sigma
                                              :car.wheels[0].I,0.4f,4.0f,"%.2f");
        if(!car.wheels.empty()){
            double wi=car.wheels[0].I;
            for(Wheel& wh:car.wheels) wh.I=wi;
        }
        dSlider({cx+130,cy+94,210,20},"Relax length",car.sigma,0.10f,1.00f,"%.2f m");
        int lx=(int)panel.x+400, ly=(int)panel.y+64;
        DrawText(TextFormat("mass: %.0f kg (GVM 3500)",car.mass),lx,ly,15,
                 Color{120,200,240,255});
        DrawText("Changes apply when you press PLAY.",lx,ly+26,13,Color{200,180,120,255});
    }
    else {                                        // ---------------- SUSP
        DrawText("Preset",(int)cx,(int)cy+4,16,Color{150,160,175,255});
        int old=suspPreset;
        GuiToggleGroup({cx+70,cy,90,26},"UNLADEN;LADEN;ROUGH",&suspPreset);
        if(suspPreset!=old) susp=phys::suspPreset(suspPreset);
        auto fs=[&](Rectangle r,const char*l,float&v,float lo,float hi,const char*fmt){
            GuiSlider(r,l,TextFormat(fmt,v),&v,lo,hi); };
        fs({cx+110,cy+40,200,18},"Rest len",  susp.rest,     0.20f,0.70f,"%.2f m");
        fs({cx+110,cy+62,200,18},"Travel",    susp.travel,   0.08f,0.45f,"%.2f m");
        fs({cx+110,cy+84,200,18},"Stiffness", susp.stiffness,8000.f,90000.f,"%.0f");
        fs({cx+110,cy+106,200,18},"Damping",  susp.damping,  1000.f,12000.f,"%.0f");
        fs({cx+110,cy+128,200,18},"Lateral K",susp.gripK,    5000.f,60000.f,"%.0f");
        int lx=(int)panel.x+400, ly=(int)panel.y+64;
        DrawText(TextFormat("preset: %s",phys::suspPresetName(suspPreset)),
                 lx,ly,15,Color{120,220,140,255});
        DrawText("Each wheel casts one ray; the spring",lx,ly+28,13,Color{120,128,140,255});
        DrawText("holds the chassis off the terrain and",lx,ly+44,13,Color{120,128,140,255});
        DrawText("sets the tyre load Fz for the model.",lx,ly+60,13,Color{120,128,140,255});
    }
}

// ----------------------------- main -----------------------------------------
int main(){
    int W=1180,H=760;
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(W,H,"Transit Mk7 - CUSTOM drivetrain (Jolt = raycasts + collision only)");
    SetWindowMinSize(1080,700);
    SetTargetFPS(60);
    GuiSetStyle(DEFAULT,TEXT_SIZE,16);

    InitAudioDevice();
    Music engSnd = LoadMusicStream(TextFormat("%sidle.ogg",GetApplicationDirectory()));
    if(engSnd.frameCount==0) engSnd = LoadMusicStream("idle.ogg");
    bool haveSnd = engSnd.frameCount>0;
    if(haveSnd){ engSnd.looping=true; PlayMusicStream(engSnd); }

    Vehicle car;
    car.external = true;              // Jolt integrates the chassis, we do the rest
    float throttle=0, clutchEng=1.0f, brake=0, steer=0;
    bool  running=true, prevRunning=true, soundOn=false;
    int   tuneTab=0, pointSel=0, gearSel=1;
    Plot  pRPM,pWheel,pSlip,pForce;

    const float MAX_STEER = 36.0f*DEG2RAD;
    const int   SUB = 8;              // drivetrain substeps per rendered frame

    // --- 3D world ---------------------------------------------------------
    const int   TN=256;
    const float TCELL=11.0f;
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

    int        suspPreset = phys::SUSP_UNLADEN;
    phys::Susp susp = phys::suspPreset(suspPreset);
    Vector3    com  = {0.16f,-0.45f,0.0f};

    // build the raycast rig from the wheel layout (no Jolt drivetrain at all)
    auto buildRig=[&](){
        if(car.wheels.empty()) return;
        float minx=1e9f,maxx=-1e9f,minz=1e9f,maxz=-1e9f;
        for(const Wheel& wh: car.wheels){
            minx=std::min(minx,(float)wh.px); maxx=std::max(maxx,(float)wh.px);
            minz=std::min(minz,(float)wh.pz); maxz=std::max(maxz,(float)wh.pz); }
        float blen=std::max((maxx-minx)+2.38f,1.2f);
        float bwid=std::max((maxz-minz)+0.25f,0.8f);
        float bhei=2.05f;
        float attachY=-bhei*0.5f+0.50f;

        std::vector<float> ox,oy,oz;
        for(const Wheel& wh: car.wheels){
            ox.push_back((float)wh.px); oy.push_back(attachY); oz.push_back((float)wh.pz); }

        susp.radius = car.wheels.empty()? 0.345f : (float)car.wheels[0].r;
        world.setSusp(susp);
        world.buildRig(ox,oy,oz,blen,bhei,bwid,(float)car.mass,
                       0.0f,spawnH,0.0f, com.x,com.y,com.z);
        car.reset();
    };

    int terrainMode=0;
    auto regenTerrain=[&](){
        gFlatTerrain = (terrainMode==1);
        for(int iz=0;iz<TN;iz++) for(int ix=0;ix<TN;ix++){
            float wx=-TSPAN*0.5f+TCELL*ix, wz=-TSPAN*0.5f+TCELL*iz;
            heights[(size_t)iz*TN+ix]=terrainH(wx,wz);
        }
        world.setHeightfield(TN,TCELL,heights);
        spawnH = heights[(size_t)(TN/2)*TN + (TN/2)] + 1.6f;
        terrain.materials[0].shader.id = rlGetShaderIdDefault();
        UnloadModel(terrain);
        terrain = BuildTerrainModel(TN,TCELL,heights);
        terrain.materials[0].shader = litShader;
        buildRig();
    };

    buildRig();

    world.addRamp(  9.0f,0.0f,  1.5f, 0.0f, 9.0f,1.7f,6.0f);
    world.addRamp(-10.0f,0.0f, -3.0f, PI,   8.0f,1.4f,5.0f);
    {
        float bx=6.0f, bz=-8.0f, hh=0.6f;
        for(int r=0;r<3;r++) for(int cc=0;cc<3-r;cc++)
            world.addCrate(bx + cc*1.3f + r*0.65f,
                           hh + r*(2.0f*hh+0.04f) + 0.1f, bz, hh, 35.0f);
        world.addCrate(10.5f,1.0f, 6.0f,0.6f,30.0f);
        world.addCrate(-5.0f,1.0f, 9.0f,0.7f,45.0f);
    }

    while(!WindowShouldClose()){
        // ---- input --------------------------------------------------------
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))   throttle=std::min(1.0f,throttle+0.04f);
        else                                      throttle=std::max(0.0f,throttle-0.06f);
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) brake=std::min(1.0f,brake+0.06f);
        else                                      brake=std::max(0.0f,brake-0.08f);
        if(IsKeyDown(KEY_SPACE))                  clutchEng=std::max(0.0f,clutchEng-0.10f);
        else                                      clutchEng=std::min(1.0f,clutchEng+0.06f);
        float steerTgt=0.0f;
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))  steerTgt=-1.0f;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) steerTgt=+1.0f;
        steer += (steerTgt-steer)*0.18f;
        if(IsKeyPressed(KEY_E)&&car.box.gear<car.box.gears()) car.box.gear++;
        if(IsKeyPressed(KEY_Q)&&car.box.gear>0) car.box.gear--;
        if(IsKeyPressed(KEY_ONE))   car.surf=0;
        if(IsKeyPressed(KEY_TWO))   car.surf=1;
        if(IsKeyPressed(KEY_THREE)) car.surf=2;
        if(IsKeyPressed(KEY_P))     running=!running;
        if(IsKeyPressed(KEY_M))     soundOn=!soundOn;
        if(IsKeyPressed(KEY_R)) { car.reset(); world.resetVehicle(0,spawnH,0);
                                  world.resetObstacles(); throttle=brake=0; clutchEng=1; }

        car.clu.engagement = clutchEng;
        if(running && !prevRunning) buildRig();
        prevRunning = running;

        // ================= the actual simulation ==========================
        if(running && world.hasVehicle()){
            float bp0[3]; world.bodyPosition(bp0);
            car.surf = surfZone(bp0[0], bp0[2]);

            const auto& wo = world.wheels();
            int N=(int)car.wheels.size();

            // 1) pull each wheel's load and contact speed out of the rig
            for(int i=0;i<N && i<(int)wo.size();++i){
                car.wheels[i].Fz       = wo[i].grounded ? wo[i].Fz : 0.0;
                car.wheels[i].vx       = wo[i].vx;
                car.wheels[i].grounded = wo[i].grounded!=0;
                // front wheels steer
                car.wheels[i].steer = (car.wheels[i].px > 0.01)
                                        ? steer*MAX_STEER : 0.0;
            }

            // 2) OUR drivetrain: engine -> clutch -> gearbox -> diffs -> tyres.
            //    Substepped because the clutch/driveline is a stiff coupling.
            float dt=1.0f/60.0f;
            for(int s=0;s<SUB;s++) car.step(dt/SUB, throttle, brake);

            // 3) hand the tyre forces back to Jolt and integrate the chassis
            std::vector<phys::RigWheelIn> in((size_t)N);
            for(int i=0;i<N;++i){
                in[i].steer = (float)car.wheels[i].steer;
                in[i].Fx    = (float)car.wheels[i].Fx;
            }
            world.stepRig(dt, in, SURFACES[car.surf].mu);

            car.v = world.forwardSpeed();

            float bp[3]; world.bodyPosition(bp);
            float half = TSPAN*0.5f;
            if(bp[1] < -10.0f || std::fabs(bp[0])>half+4.0f || std::fabs(bp[2])>half+4.0f){
                world.resetVehicle(0,spawnH,0); car.reset();
            }
        }

        // ---- telemetry ----------------------------------------------------
        int mon=-1;
        for(int i=0;i<(int)car.wheels.size();++i) if(car.wheels[i].driven){ mon=i; break; }
        if(mon<0) mon=0;
        bool haveW = !car.wheels.empty();

        double rpm  = car.eng.omega*RAD2RPM;
        double kmh  = car.v*3.6;
        double n    = car.box.n();
        double carrier=0.0; int nd=car.drivenCount();
        if(nd>0){ for(const Wheel& wh:car.wheels) if(wh.driven) carrier+=wh.omega; carrier/=nd; }
        double wheelRPM = carrier*n*RAD2RPM;
        double monFz = haveW? car.wheels[mon].Fz : 0.0;
        double monK  = haveW? car.wheels[mon].kappa : 0.0;
        double monFx = haveW? car.wheels[mon].Fx : 0.0;
        double kP    = car.tire.kappaPeak(SURFACES[car.surf].mu, std::max(monFz,1.0));
        if(running){
            pRPM.push((float)rpm); pWheel.push((float)wheelRPM);
            pSlip.push((float)monK); pForce.push((float)monFx);
        }

        if(haveSnd){
            double frac = (rpm - car.eng.idleRPM)/(car.eng.redline - car.eng.idleRPM);
            frac = clampd(frac, 0.0, 1.0);
            SetMusicPitch(engSnd, (float)(0.85 + frac*2.15));
            bool silent = !soundOn || car.eng.stalled || !running;
            SetMusicVolume(engSnd, silent?0.0f:1.0f);
            UpdateMusicStream(engSnd);
        }

        // ===================== render ======================================
        W=GetScreenWidth(); H=GetScreenHeight();
        const float PANELW=300.0f, M=10.0f;
        Rectangle scene={PANELW+M, M, (float)W-(PANELW+2*M), (float)(H-3*M)*0.62f};
        Rectangle lower={scene.x, scene.y+scene.height+M, scene.width,
                         (float)H-(scene.y+scene.height+2*M)};

        BeginDrawing();
        ClearBackground(Color{18,20,25,255});

        DrawRectangle(0,0,(int)PANELW,H,Color{26,28,34,255});
        DrawText("CUSTOM DRIVETRAIN",20,14,18,Color{120,220,140,255});
        DrawText("Jolt: raycasts + collision only",20,36,13,Color{120,128,140,255});

        GuiSlider({110,62,150,22},"Throttle",TextFormat("%.2f",throttle),&throttle,0,1);
        GuiSlider({110,92,150,22},"Clutch",  TextFormat("%.2f",clutchEng),&clutchEng,0,1);
        GuiSlider({110,122,150,22},"Brake",  TextFormat("%.2f",brake),&brake,0,1);

        DrawText("GEAR",20,160,16,Color{150,160,175,255});
        if(GuiButton({90,156,40,26},"-")&&car.box.gear>0) car.box.gear--;
        const char* gname = car.box.gear==0?"N":TextFormat("%d",car.box.gear);
        DrawText(gname,150,160,20,RAYWHITE);
        if(GuiButton({180,156,40,26},"+")&&car.box.gear<car.box.gears()) car.box.gear++;

        DrawText("SURFACE",20,200,16,Color{150,160,175,255});
        int surfSel=car.surf;
        GuiToggleGroup({100,196,55,26},"DRY;WET;ICE",&surfSel);
        car.surf=surfSel;

        Color simBtnCol = running ? Color{200,80,70,255} : Color{90,180,110,255};
        int prevBase  = GuiGetStyle(BUTTON,BASE_COLOR_NORMAL);
        GuiSetStyle(BUTTON,BASE_COLOR_NORMAL,ColorToInt(simBtnCol));
        if(GuiButton({20,238,170,30}, running?"#132#STOP  (P)":"#131#PLAY  (P)"))
            running=!running;
        GuiSetStyle(BUTTON,BASE_COLOR_NORMAL,prevBase);
        if(GuiButton({200,238,80,30},"Reset")){ car.reset(); world.resetVehicle(0,spawnH,0);
            world.resetObstacles(); throttle=brake=0; clutchEng=1; }
        DrawText(running?"SIM: RUNNING":"SIM: STOPPED (tunable)",20,276,14,
                 running?Color{120,220,140,255}:Color{240,200,90,255});
        if(GuiButton({200,272,80,26}, soundOn?"#122#Sound":"#123#Muted")) soundOn=!soundOn;

        DrawText("W throttle  S brake  A/D steer",20,304,13,Color{120,128,140,255});
        DrawText("SPACE clutch  Q/E shift  R reset",20,322,13,Color{120,128,140,255});

        int ry=350;
        auto line=[&](const char*k,const char*v,Color c){
            DrawText(k,20,ry,16,Color{150,160,175,255});
            DrawText(v,150,ry,16,c); ry+=23; };
        line("Engine", car.eng.stalled?"STALLED":TextFormat("%.0f rpm",rpm),
                       car.eng.stalled?Color{240,90,80,255}:RAYWHITE);
        line("Speed",  TextFormat("%.1f km/h",kmh),RAYWHITE);
        line("Clutch", car.clu.lockedish?"LOCKED":"slipping",
                       car.clu.lockedish?Color{120,220,140,255}:Color{240,200,90,255});
        line("Reduction", car.box.gear? TextFormat("%.2f :1",n):"neutral",RAYWHITE);
        line("Surface", SURFACES[car.surf].name,
             car.surf==0?Color{120,220,140,255}:
             car.surf==1?Color{110,170,230,255}:Color{200,220,240,255});

        // per-wheel telemetry table
        ry+=6;
        DrawText("WHEEL   Fz(N)   slip     rpm",20,ry,13,Color{150,160,175,255}); ry+=18;
        for(int i=0;i<(int)car.wheels.size();++i){
            const Wheel& wh=car.wheels[i];
            Color c = !wh.grounded ? Color{240,160,90,255}
                    : (std::fabs(wh.kappa)>kP*1.05 ? Color{240,90,80,255} : RAYWHITE);
            DrawText(TextFormat("%d %s  %6.0f  %+5.2f  %6.0f",
                     i+1, wh.driven?"D":" ", wh.Fz, wh.kappa,
                     wh.omega*RAD2RPM), 20,ry,13,c);
            ry+=16;
        }
        if(car.eng.stalled)
            DrawText("engine dead - clutch in or\nneutral to restart",20,ry+6,13,
                     Color{240,90,80,255});

        // --- 3D view --------------------------------------------------------
        DrawScene3D(scene, world, car, terrain, susp, com);

        int tmode=terrainMode;
        GuiToggleGroup({scene.x+scene.width-150,scene.y+8,68,22},"NOISE;FLAT",&tmode);
        if(tmode!=terrainMode){ terrainMode=tmode; regenTerrain(); }

        float gR=std::min(70.0f, scene.width*0.075f);
        DrawGauge({scene.x+gR+14, scene.y+scene.height-gR-14}, gR,
                  rpm,5000,"RPM x1000",
                  car.eng.stalled?"DEAD":TextFormat("%.1f",rpm/1000.0),
                  car.eng.redline/5000.0);
        DrawGauge({scene.x+scene.width-gR-14, scene.y+scene.height-gR-14}, gR,
                  std::fabs(kmh),180,"km/h",TextFormat("%.0f",std::fabs(kmh)),1.0);

        // --- lower: plots while running, tuning while stopped ---------------
        if(running){
            float pw=(lower.width-20)*0.5f, ph=(lower.height-14)*0.5f, py=lower.y;
            DrawPlot({lower.x,py,pw,ph},pRPM,0,5000,Color{240,200,90,255},"engine rpm");
            DrawPlot({lower.x,py,pw,ph},pWheel,0,5000,Color{120,200,240,255},
                     "wheel-equiv rpm",NAN);
            DrawText("engine vs wheel-equiv rpm (gap = clutch slip)",
                     (int)lower.x+6,(int)(py+ph-18),12,Color{120,128,140,255});
            DrawPlot({lower.x+pw+20,py,pw,ph},pSlip,-0.5f,0.5f,Color{240,120,90,255},
                     "monitored slip k",0.0f);
            DrawPlot({lower.x,py+ph+14,pw,ph},pForce,-16000,16000,
                     Color{140,220,150,255},"tyre Fx (N)",0.0f);
            Rectangle info={lower.x+pw+20,py+ph+14,pw,ph};
            DrawRectangleRec(info,Color{24,26,32,255});
            DrawRectangleLinesEx(info,1,Color{60,64,74,255});
            DrawText("DRIVELINE",(int)info.x+8,(int)info.y+6,14,Color{150,160,175,255});
            DrawText(TextFormat("engine  %.0f rpm  %s",rpm,
                     car.eng.stalled?"(dead)":""),(int)info.x+8,(int)info.y+26,13,RAYWHITE);
            DrawText(TextFormat("clutch  %.0f%% engaged, %s",clutchEng*100.0f,
                     car.clu.lockedish?"locked":"slipping"),
                     (int)info.x+8,(int)info.y+44,13,RAYWHITE);
            DrawText(TextFormat("gearbox %s  reduction %.2f",
                     car.box.gear?TextFormat("gear %d",car.box.gear):"neutral",n),
                     (int)info.x+8,(int)info.y+62,13,RAYWHITE);
            DrawText(TextFormat("diffs   %d   driven wheels %d",
                     (int)car.diffs.size(),nd),(int)info.x+8,(int)info.y+80,13,RAYWHITE);
            DrawText(TextFormat("slip peak k = %.3f on %s",kP,SURFACES[car.surf].name),
                     (int)info.x+8,(int)info.y+98,13,Color{120,128,140,255});
        } else {
            DrawTuning(lower, car, tuneTab, pointSel, gearSel, susp, suspPreset);
        }

        EndDrawing();
    }

    if(haveSnd) UnloadMusicStream(engSnd);
    CloseAudioDevice();
    terrain.materials[0].shader.id = rlGetShaderIdDefault();
    UnloadShader(litShader);
    UnloadModel(terrain);
    CloseWindow();
    return 0;
}
