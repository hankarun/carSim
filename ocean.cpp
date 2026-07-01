// =============================================================================
//  ocean.cpp -- FFT ocean implementation (see ocean.h).
// =============================================================================
#include "ocean.h"
#include "fft.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <random>

static const float OPI=3.14159265358979323846f;

// wavevector components for grid cell (n,m), centred on k=0
static inline void waveVec(const Ocean& o,int n,int m,float& kx,float& kz){
    kx = 2.0f*OPI*(n - o.N/2) / o.L;
    kz = 2.0f*OPI*(m - o.N/2) / o.L;
}

// Phillips spectrum P(k)
static float phillips(const Ocean& o,float kx,float kz){
    float k2 = kx*kx + kz*kz;
    if(k2 < 1e-8f) return 0.0f;
    float k  = std::sqrt(k2);
    float Lw = o.windSpeed*o.windSpeed / o.gravity;   // largest wave from wind
    float wx = std::cos(o.windDir), wz = std::sin(o.windDir);
    float kdotw = (kx*wx + kz*wz) / k;                // k_hat . w_hat
    float dir = std::pow(std::fabs(kdotw), o.align);
    float l   = o.L / 1000.0f;                         // suppress tiny waves
    float p = o.A * std::exp(-1.0f/(k2*Lw*Lw)) / (k2*k2) * dir
                  * std::exp(-k2*l*l);
    return p;
}

void oceanInit(Ocean& o){
    const int N=o.N;
    o.h0.assign((size_t)N*N,{0,0});
    o.h0conj.assign((size_t)N*N,{0,0});
    o.omega.assign((size_t)N*N,0.0f);
    o.hkt.assign((size_t)N*N,{0,0});
    o.height.assign((size_t)N*N,0.0f);

    std::mt19937 rng(1337u);
    std::normal_distribution<float> gauss(0.0f,1.0f);

    for(int m=0;m<N;++m) for(int n=0;n<N;++n){
        float kx,kz; waveVec(o,n,m,kx,kz);
        // h0(k)
        float p = phillips(o,kx,kz);
        float er=gauss(rng), ei=gauss(rng);
        o.h0[(size_t)m*N+n] = std::complex<float>(er,ei) * (0.70710678f*std::sqrt(p));
        // conj(h0(-k))
        float p2 = phillips(o,-kx,-kz);
        float er2=gauss(rng), ei2=gauss(rng);
        std::complex<float> h0m = std::complex<float>(er2,ei2) * (0.70710678f*std::sqrt(p2));
        o.h0conj[(size_t)m*N+n] = std::conj(h0m);
        // dispersion (deep water)
        float k = std::sqrt(kx*kx + kz*kz);
        o.omega[(size_t)m*N+n] = std::sqrt(o.gravity*k);
    }

    // The inverse FFT carries a 1/N^2 factor and the absolute Phillips scale is
    // arbitrary, so measure the resulting spatial RMS at t=0 and rescale the
    // spectrum to hit the requested wave height (in metres) -- makes the
    // "waveHeight" slider physically meaningful regardless of N / wind.
    for(int i=0;i<N*N;++i) o.hkt[i]=o.h0[i]+o.h0conj[i];
    oc::fft2d(o.hkt,N,true);
    double sum2=0.0;
    for(int m=0;m<N;++m) for(int n=0;n<N;++n){
        float sign=((n+m)&1)?-1.0f:1.0f;
        float v=o.hkt[(size_t)m*N+n].real()*sign;
        sum2+=(double)v*v;
    }
    double rms=std::sqrt(sum2/((double)N*N));
    float scale=(rms>1e-9)?(float)(o.waveHeight/rms):0.0f;
    for(int i=0;i<N*N;++i){ o.h0[i]*=scale; o.h0conj[i]*=scale; }
}

void oceanBuildMesh(Ocean& o){
    const int N=o.N;
    Mesh m{};
    m.vertexCount   = N*N;
    m.triangleCount = (N-1)*(N-1)*2;
    m.vertices = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.normals  = (float*)MemAlloc(m.vertexCount*3*sizeof(float));
    m.colors   = (unsigned char*)MemAlloc(m.vertexCount*4*sizeof(unsigned char));
    m.indices  = (unsigned short*)MemAlloc(m.triangleCount*3*sizeof(unsigned short));
    float cell = o.L/N;
    for(int iz=0;iz<N;++iz) for(int ix=0;ix<N;++ix){
        int v=iz*N+ix;
        m.vertices[v*3+0]=cell*ix;          // local patch coords [0,L)
        m.vertices[v*3+1]=0.0f;
        m.vertices[v*3+2]=cell*iz;
        m.normals[v*3+0]=0; m.normals[v*3+1]=1; m.normals[v*3+2]=0;
        m.colors[v*4+0]=18; m.colors[v*4+1]=62; m.colors[v*4+2]=92; m.colors[v*4+3]=255;
    }
    int t=0;
    for(int iz=0;iz<N-1;++iz) for(int ix=0;ix<N-1;++ix){
        unsigned short a=iz*N+ix, b=iz*N+ix+1, c=(iz+1)*N+ix, d=(iz+1)*N+ix+1;
        m.indices[t++]=a; m.indices[t++]=c; m.indices[t++]=b;
        m.indices[t++]=b; m.indices[t++]=c; m.indices[t++]=d;
    }
    UploadMesh(&m,true);                     // dynamic: buffers updated each frame
    o.mesh  = m;
    o.model = LoadModelFromMesh(m);
    o.built = true;
}

void oceanUpdate(Ocean& o, float t){
    const int N=o.N;
    // time-evolve the spectrum: h(k,t) = h0 e^{iwt} + conj(h0(-k)) e^{-iwt}
    for(int i=0;i<N*N;++i){
        float w = o.omega[i]*t;
        std::complex<float> ep(std::cos(w), std::sin(w));
        std::complex<float> em(std::cos(w),-std::sin(w));
        o.hkt[i] = o.h0[i]*ep + o.h0conj[i]*em;
    }
    // inverse FFT k-space -> spatial
    oc::fft2d(o.hkt, N, true);
    // sign flip (-1)^(n+m) compensates the k-centred indexing
    for(int m=0;m<N;++m) for(int n=0;n<N;++n){
        float sign = ((n+m)&1) ? -1.0f : 1.0f;
        o.height[(size_t)m*N+n] = o.hkt[(size_t)m*N+n].real()*sign;
    }

    if(!o.built) return;
    // rewrite mesh vertex heights + normals (central differences) and push to GPU
    float cell=o.L/N;
    auto H=[&](int ix,int iz)->float{
        ix=((ix%N)+N)%N; iz=((iz%N)+N)%N;     // wrap (periodic)
        return o.height[(size_t)iz*N+ix];
    };
    for(int iz=0;iz<N;++iz) for(int ix=0;ix<N;++ix){
        int v=iz*N+ix;
        o.mesh.vertices[v*3+1]=o.height[(size_t)iz*N+ix];
        float dx=H(ix+1,iz)-H(ix-1,iz);
        float dz=H(ix,iz+1)-H(ix,iz-1);
        Vector3 nrm=Vector3Normalize({-dx, 2.0f*cell, -dz});
        o.mesh.normals[v*3+0]=nrm.x; o.mesh.normals[v*3+1]=nrm.y; o.mesh.normals[v*3+2]=nrm.z;
    }
    UpdateMeshBuffer(o.mesh, 0, o.mesh.vertices, N*N*3*sizeof(float), 0); // positions
    UpdateMeshBuffer(o.mesh, 2, o.mesh.normals,  N*N*3*sizeof(float), 0); // normals
}

float oceanSampleHeight(const Ocean& o, float wx, float wz){
    const int N=o.N;
    float u = wx / o.L * N;
    float v = wz / o.L * N;
    int n0 = (int)std::floor(u), m0 = (int)std::floor(v);
    float fu = u - n0, fv = v - m0;
    auto H=[&](int ix,int iz)->float{
        ix=((ix%N)+N)%N; iz=((iz%N)+N)%N;
        return o.height[(size_t)iz*N+ix];
    };
    float h00=H(n0,m0),   h10=H(n0+1,m0);
    float h01=H(n0,m0+1), h11=H(n0+1,m0+1);
    float a=h00+(h10-h00)*fu;
    float b=h01+(h11-h01)*fu;
    return a+(b-a)*fv;
}

void oceanUnload(Ocean& o){
    if(o.built){ UnloadModel(o.model); o.built=false; }
}
