// Headless acceptance harness for the vsim API + Jolt raycast rig.
// Mirrors van_main.cpp's loop exactly, minus rendering.  A 1D harness over
// sim.cpp alone cannot catch rig bugs, so this drives the real phys::World.
//
//   clang++ -DJPH_DEBUG_RENDERER -DJPH_OBJECT_STREAM -DJPH_PROFILE_ENABLED \
//           -D_DEBUG -std=gnu++17 -O1 -I. -Ibuild/_deps/joltphysics-src/Build/.. \
//           tools/apitest.cpp sim.cpp drivetrain.cpp physics.cpp \
//           -Lbuild/_deps/joltphysics-build -lJolt -o /tmp/apitest && /tmp/apitest
//
// The -D flags must match build/compile_commands.json or Jolt asserts on a
// compile-flag mismatch at startup.
#include "../sim.h"
#include "../drivetrain.h"
#include "../physics.h"
#include <cstdio>
#include <cmath>
#include <vector>

static const int   TN   = 64;
static const float TCELL= 11.0f;

struct Rig {
    vsim::ManualDrivetrain van;
    vsim::IVehicleSim&     sim;
    vsim::Layout           rig;
    phys::World            world;
    std::vector<float>     heights;
    float                  spawnH = 0.0f;

    Rig() : sim(van), heights((size_t)TN*TN, 0.0f) {
        van.external = true;
        world.setHeightfield(TN,TCELL,heights);
        spawnH = 1.6f;
        rig = sim.layout();
        std::vector<float> ox,oy,oz;
        for(const vsim::WheelSpec& w : rig.wheels){
            ox.push_back((float)w.px); oy.push_back((float)w.py); oz.push_back((float)w.pz);
        }
        phys::Susp s = phys::suspPreset(phys::SUSP_UNLADEN);
        s.radius = (float)rig.wheels[0].radius;
        world.setSusp(s);
        world.buildRig(ox,oy,oz,
                       (float)rig.body.length,(float)rig.body.height,(float)rig.body.width,
                       (float)rig.body.mass, 0,spawnH,0,
                       (float)rig.body.comX,(float)rig.body.comY,(float)rig.body.comZ);
        sim.reset();
    }

    void frame(double throttle,double brake,double clutch,int gear,double mu=1.0){
        const float dt = 1.0f/60.0f;
        int N = sim.wheelCount();
        const auto& wo = world.wheels();
        std::vector<vsim::ContactIn> contacts((size_t)N);
        for(int i=0;i<N;++i){
            if(i<(int)wo.size()){
                contacts[i].Fz = wo[i].Fz; contacts[i].vx = wo[i].vx;
                contacts[i].grounded = wo[i].grounded!=0;
            }
            contacts[i].mu = mu;
        }
        vsim::StepInput in;
        in.cmd.throttle=throttle; in.cmd.brake=brake;
        in.cmd.clutch=clutch;     in.cmd.gear=gear;
        in.bodySpeed = world.forwardSpeed();
        in.contacts = contacts.data(); in.contactCount = N;
        sim.step(dt, in);

        const vsim::WheelOut* w = sim.wheelOutputs();
        std::vector<phys::RigWheelIn> rin((size_t)N);
        for(int i=0;i<N;++i){ rin[i].steer=(float)w[i].steer; rin[i].Fx=(float)w[i].Fx; }
        world.stepRig(dt, rin, (float)mu);
    }

    void settle(){ for(int i=0;i<180;i++) frame(0,0,1,0); sim.reset(); }
    float x() const { float p[3]; world.bodyPosition(p); return p[0]; }
};

static int fails = 0;
static void check(bool ok,const char* what,const char* detail){
    std::printf("%s  %-52s %s\n", ok?"PASS":"FAIL", what, detail);
    if(!ok) fails++;
}

int main(){
    // ---- A: neutral, hands off -> must not creep -------------------------
    {
        Rig r; r.settle();
        float x0 = r.x();
        for(int i=0;i<600;i++) r.frame(0,0,1,0);      // 10 s in neutral
        float drift = r.x()-x0;
        double v = r.sim.telemetry().speed;
        char d[128]; std::snprintf(d,sizeof d,"drift %+.3f m, v %+.3f m/s",drift,v);
        check(std::fabs(drift)<0.20 && std::fabs(v)<0.10, "neutral: no creep", d);
    }

    // ---- B: dump the clutch from rest -> stalls in every gear -------------
    for(int g=1; g<=6; ++g){
        Rig r; r.settle();
        for(int i=0;i<240;i++) r.frame(1.0,0,1.0,g);  // clutch fully out at once
        vsim::Telemetry t = r.sim.telemetry();
        char d[128]; std::snprintf(d,sizeof d,"rpm %.0f, v %.2f m/s",t.engineRPM,t.speed);
        check(t.engineStalled, ("dump clutch in gear " + std::to_string(g) + ": stalls").c_str(), d);
    }

    // ---- C: feather the clutch in 1st -> launches without stalling --------
    {
        Rig r; r.settle();
        double clutch = 0.0;
        for(int i=0;i<600;i++){
            clutch = std::min(1.0, clutch + 1.0/240.0);   // 4 s to fully engage
            r.frame(0.55, 0, clutch, 1);
        }
        vsim::Telemetry t = r.sim.telemetry();
        char d[128]; std::snprintf(d,sizeof d,"v %.2f m/s, rpm %.0f, %s",
                                   t.speed,t.engineRPM,t.engineStalled?"stalled":"running");
        check(!t.engineStalled && t.speed>2.0, "feathered launch in 1st", d);
    }

    // ---- D: telemetry/layout sanity --------------------------------------
    {
        Rig r;
        char d[160];
        std::snprintf(d,sizeof d,"L %.2f W %.2f H %.2f, %d wheels, %d driven, com %.2f/%.2f",
                      r.rig.body.length,r.rig.body.width,r.rig.body.height,
                      (int)r.rig.wheels.size(),r.sim.telemetry().drivenWheels,
                      r.rig.body.comX,r.rig.body.comY);
        bool ok = std::fabs(r.rig.body.length-5.68)<1e-6
               && std::fabs(r.rig.body.width -1.97)<1e-6
               && std::fabs(r.rig.body.height-2.05)<1e-6
               && r.rig.wheels.size()==4
               && std::fabs(r.rig.wheels[0].py-(-0.525))<1e-6
               && r.rig.wheels[0].steerable && !r.rig.wheels[2].steerable;
        check(ok, "layout() matches the old hard-coded rig", d);
    }

    std::printf("\n%s (%d failure%s)\n", fails? "FAILED":"ALL PASSED", fails, fails==1?"":"s");
    return fails? 1:0;
}
