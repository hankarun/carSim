// =============================================================================
//  Ship simulator -- FFT ocean + buoyancy-from-triangle-centers + a propeller.
//
//  A second, standalone executable alongside the car sim.  The sea is a periodic
//  FFT (Phillips spectrum) patch tiled to the horizon (ocean.*); the ship is a
//  triangle hull floating via per-facet hydrostatic buoyancy and driven by a
//  motor-spun propeller with a rudder (ship.*).
//
//  Controls:  W/S throttle   A/D rudder   P pause   R reset   X stop throttle
//  Build with the accompanying CMakeLists.txt (shipsim target).
// =============================================================================
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "ocean.h"
#include "ship.h"

#include <cmath>
#include <algorithm>

// --- single directional-light shader (same pattern as the car sim) ----------
static const char* OCEAN_VS =
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
static const char* OCEAN_FS =
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

// ----------------------------- orbit camera ---------------------------------
static Vector3 gCamPos = {20.0f,16.0f,28.0f};
static float   gOrbitYaw=2.4f, gOrbitPitch=0.45f, gOrbitDist=34.0f;

static Camera3D updateCamera(Vector3 target, bool allowInput){
    if(allowInput){
        if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)||IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
            Vector2 dd=GetMouseDelta();
            gOrbitYaw   -= dd.x*0.006f;
            gOrbitPitch += dd.y*0.006f;
            gOrbitPitch  = std::clamp(gOrbitPitch,-1.45f,1.45f);  // allow looking up from below
        }
        float wheel=GetMouseWheelMove();
        if(wheel!=0) gOrbitDist=std::clamp(gOrbitDist-wheel*3.0f,8.0f,140.0f);
    }
    Vector3 tgt=Vector3Add(target,(Vector3){0,1.5f,0});
    Vector3 off={ std::cos(gOrbitPitch)*std::sin(gOrbitYaw),
                  std::sin(gOrbitPitch),
                  std::cos(gOrbitPitch)*std::cos(gOrbitYaw) };
    Vector3 want=Vector3Add(tgt,Vector3Scale(off,gOrbitDist));
    gCamPos=Vector3Lerp(gCamPos,want,0.20f);
    Camera3D cam{};
    cam.position=gCamPos; cam.target=tgt;
    cam.up={0,1,0}; cam.fovy=55.0f; cam.projection=CAMERA_PERSPECTIVE;
    return cam;
}

static void drawShip(const Ship& s){
    // hull: lit model, transform = rotation * translation
    Ship& m=const_cast<Ship&>(s);
    m.model.transform=MatrixMultiply(QuaternionToMatrix(s.orient),
                                     MatrixTranslate(s.pos.x,s.pos.y,s.pos.z));
    rlDisableBackfaceCulling();
    DrawModel(m.model,{0,0,0},1.0f,WHITE);
    rlEnableBackfaceCulling();

    // propeller + rudder: immediate-mode, in the ship's frame
    Vector3 ax; float an; QuaternionToAxisAngle(s.orient,&ax,&an);
    rlPushMatrix();
    rlTranslatef(s.pos.x,s.pos.y,s.pos.z);
    if(Vector3Length(ax)>0.001f) rlRotatef(an*RAD2DEG,ax.x,ax.y,ax.z);

    // propeller (spins about body +X)
    rlPushMatrix();
    rlTranslatef(s.propPos.x,s.propPos.y,s.propPos.z);
    rlRotatef(s.propAngle*RAD2DEG,1,0,0);
    DrawSphere({0,0,0},0.20f,Color{60,64,72,255});
    for(int b=0;b<4;++b){
        rlPushMatrix();
        rlRotatef(b*90.0f,1,0,0);
        DrawCube({0,0.42f,0},0.06f,0.80f,0.18f,Color{90,96,104,255});
        rlPopMatrix();
    }
    rlPopMatrix();

    // rudder (rotates about body +Y)
    rlPushMatrix();
    rlTranslatef(s.rudderPos.x,s.rudderPos.y,s.rudderPos.z);
    rlRotatef(s.rudderAngle*RAD2DEG,0,1,0);
    DrawCube({-0.2f,0,0},0.10f,0.9f,0.9f,Color{70,74,82,255});
    rlPopMatrix();

    rlPopMatrix();
}

int main(){
    SetConfigFlags(FLAG_MSAA_4X_HINT|FLAG_WINDOW_RESIZABLE);
    InitWindow(1280,720,"Ship Simulator - FFT ocean + buoyancy");
    SetWindowMinSize(1000,640);
    SetTargetFPS(60);

    Ocean ocean;
    oceanInit(ocean);
    oceanBuildMesh(ocean);

    Ship ship;
    shipInit(ship);
    shipBuildMesh(ship);

    // lit shader shared by ocean + hull
    Shader lit=LoadShaderFromMemory(OCEAN_VS,OCEAN_FS);
    Vector3 ld=Vector3Normalize({-0.55f,-1.0f,-0.35f});
    Vector3 lc={1.0f,0.97f,0.90f};
    float   amb=0.40f;
    SetShaderValue(lit,GetShaderLocation(lit,"lightDir"),  &ld,SHADER_UNIFORM_VEC3);
    SetShaderValue(lit,GetShaderLocation(lit,"lightColor"),&lc,SHADER_UNIFORM_VEC3);
    SetShaderValue(lit,GetShaderLocation(lit,"ambient"),   &amb,SHADER_UNIFORM_FLOAT);
    ocean.model.materials[0].shader=lit;
    ship.model.materials[0].shader =lit;

    float  simTime=0.0f;
    bool   running=true;
    bool   debugDraw=false;

    // remember spectrum params to know when to rebuild the FFT spectrum
    float prevA=ocean.waveHeight, prevWind=ocean.windSpeed, prevDir=ocean.windDir;

    while(!WindowShouldClose()){
        Rectangle panel={10,10,300,300};
        bool overPanel=CheckCollisionPointRec(GetMousePosition(),panel);

        // ---- input ----
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))   ship.throttle=std::min(1.0f,ship.throttle+0.015f);
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) ship.throttle=std::max(-1.0f,ship.throttle-0.015f);
        float rt=0.0f;
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))  rt-=1.0f;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) rt+=1.0f;
        ship.rudder=rt;
        if(IsKeyPressed(KEY_X)) ship.throttle=0.0f;
        if(IsKeyPressed(KEY_P)) running=!running;
        if(IsKeyPressed(KEY_R)) shipReset(ship);
        if(IsKeyPressed(KEY_G)) debugDraw=!debugDraw;
        ship.recordDebug=debugDraw;

        // ---- simulate ----
        if(running){
            float frame=GetFrameTime();
            if(frame>1.0f/20.0f) frame=1.0f/20.0f;   // clamp big hitches
            simTime+=frame;
            oceanUpdate(ocean,simTime);
            const int SUB=8;
            float h=frame/SUB;
            for(int i=0;i<SUB;++i) shipStep(ship,ocean,h);
        }

        // ---- render ----
        Camera3D cam=updateCamera(ship.pos,!overPanel);
        BeginDrawing();
        ClearBackground(Color{150,190,225,255});
        BeginMode3D(cam);

        // tile the periodic ocean patch around the ship
        const int R=3;
        float baseX=std::floor(ship.pos.x/ocean.L)*ocean.L;
        float baseZ=std::floor(ship.pos.z/ocean.L)*ocean.L;
        for(int tj=-R;tj<=R;++tj) for(int ti=-R;ti<=R;++ti)
            DrawModel(ocean.model,{baseX+ti*ocean.L,0,baseZ+tj*ocean.L},1.0f,WHITE);

        drawShip(ship);

        // debug: sample points + force vectors (recorded in the last sub-step)
        if(debugDraw){
            const float FS=0.00015f;          // force -> metres for the arrows
            // draw the overlay with depth-test off so arrows aren't hidden
            // inside the hull / under the water
            rlDrawRenderBatchActive();
            rlDisableDepthTest();
            for(const DbgArrow& a : ship.dbg){
                if(a.kind==5){ DrawSphere(a.p,0.05f,Color{120,120,130,160}); continue; }
                Color c=(a.kind==2)?RED:(a.kind==3)?GOLD:(a.kind==4)?ORANGE:
                        Color{90,200,255,255};   // buoyancy = light blue
                DrawSphere(a.p,0.06f,c);
                Vector3 tip=Vector3Add(a.p,Vector3Scale(a.f,FS));
                DrawLine3D(a.p,tip,c);
                DrawSphere(tip,0.04f,c);          // arrow tip marker
            }
            rlDrawRenderBatchActive();
            rlEnableDepthTest();
        }
        EndMode3D();

        // ---- HUD ----
        float spd=shipForwardSpeed(ship);
        float rpm=ship.propOmega*60.0f/(2.0f*PI);
        int W=GetScreenWidth();
        DrawRectangle(W-230,10,220,118,Color{0,0,0,120});
        DrawText(TextFormat("Speed  %.1f m/s  (%.0f kn)",spd,spd*1.94384f),W-220,18,18,RAYWHITE);
        DrawText(TextFormat("Throttle  %+.0f %%",ship.throttle*100.0f),W-220,42,18,RAYWHITE);
        DrawText(TextFormat("Prop RPM  %.0f",rpm),W-220,66,18,RAYWHITE);
        DrawText(TextFormat("Rudder  %+.0f deg",ship.rudderAngle*RAD2DEG),W-220,90,18,RAYWHITE);
        if(!running) DrawText("PAUSED",W/2-50,12,28,Color{240,200,90,255});
        DrawText("W/S throttle  A/D rudder  P pause  R reset  G debug",12,GetScreenHeight()-26,18,
                 Color{235,235,245,255});
        if(debugDraw){
            int lx=W-230, ly=140;
            DrawRectangle(lx,ly,220,110,Color{0,0,0,120});
            DrawText("DEBUG",lx+10,ly+6,18,RAYWHITE);
            DrawText("buoyancy",lx+28,ly+30,16,Color{90,200,255,255});
            DrawText("thrust",  lx+28,ly+50,16,RED);
            DrawText("rudder",  lx+28,ly+70,16,GOLD);
            DrawText("hull drag",lx+28,ly+90,16,ORANGE);
            DrawCircle(lx+18,ly+38,4,Color{90,200,255,255});
            DrawCircle(lx+18,ly+58,4,RED);
            DrawCircle(lx+18,ly+78,4,GOLD);
            DrawCircle(lx+18,ly+98,4,ORANGE);
        }

        // ---- tuning panel ----
        DrawRectangleRec(panel,Color{26,28,34,210});
        DrawRectangleLinesEx(panel,1,Color{60,64,74,255});
        DrawText("OCEAN / SHIP",(int)panel.x+12,(int)panel.y+8,16,Color{120,220,140,255});
        // leave room left of each bar for its label and room right for the value
        float y=panel.y+34; const float gx=panel.x+78, gw=140;
        GuiSlider({gx,y,gw,18},"Wave",TextFormat("%.2f m",ocean.waveHeight),&ocean.waveHeight,0.05f,3.0f); y+=26;
        GuiSlider({gx,y,gw,18},"Wind",TextFormat("%.0f",ocean.windSpeed),&ocean.windSpeed,6.0f,45.0f); y+=26;
        GuiSlider({gx,y,gw,18},"Dir",TextFormat("%.2f",ocean.windDir),&ocean.windDir,0.0f,6.28f); y+=26;
        GuiSlider({gx,y,gw,18},"Buoy",TextFormat("%.0f",ship.buoyDensity),&ship.buoyDensity,4000.0f,16000.0f); y+=26;
        GuiSlider({gx,y,gw,18},"LinDrag",TextFormat("%.0f",ship.linDrag),&ship.linDrag,200.0f,6000.0f); y+=26;
        GuiSlider({gx,y,gw,18},"AngDrag",TextFormat("%.0f",ship.angDrag),&ship.angDrag,10000.0f,200000.0f); y+=26;
        GuiSlider({gx,y,gw,18},"Thrust",TextFormat("%.0f",ship.thrustK),&ship.thrustK,20.0f,300.0f); y+=26;
        GuiSlider({gx,y,gw,18},"PropMax",TextFormat("%.0f",ship.maxPropOmega),&ship.maxPropOmega,60.0f,400.0f); y+=26;

        EndDrawing();

        // rebuild the FFT spectrum if a wave/wind slider moved
        if(ocean.waveHeight!=prevA||ocean.windSpeed!=prevWind||ocean.windDir!=prevDir){
            oceanInit(ocean);
            prevA=ocean.waveHeight; prevWind=ocean.windSpeed; prevDir=ocean.windDir;
        }
    }

    UnloadShader(lit);
    shipUnload(ship);
    oceanUnload(ocean);
    CloseWindow();
    return 0;
}
