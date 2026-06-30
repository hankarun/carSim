// =============================================================================
//  ocean.h -- FFT ocean (Tessendorf-style, height-only).
//
//  A periodic NxN patch of sea synthesised from a Phillips spectrum.  Each frame
//  the time-evolved spectrum is inverse-FFT'd to a vertical-height grid; surface
//  normals come from central differences of that grid.  The patch is periodic so
//  it tiles seamlessly to give an infinite-looking sea.
//
//  The raylib mesh is built once (dynamic buffers) and its positions/normals are
//  pushed to the GPU each frame via UpdateMeshBuffer.
// =============================================================================
#pragma once
#include "raylib.h"
#include <complex>
#include <vector>

struct Ocean {
    int   N    = 128;          // grid resolution (power of two)
    float L    = 128.0f;       // patch size in metres == tiling period
    float A    = 1.0f;          // Phillips constant (cancels under normalization)
    float waveHeight = 0.5f;    // target RMS wave height in metres (the tuning knob)
    float windSpeed = 26.0f;   // m/s
    float windDir   = 0.4f;    // radians (wind direction in XZ plane)
    float align     = 2.0f;    // |k.w|^align directional sharpening
    float gravity   = 9.81f;

    // precomputed spectrum (size N*N, indexed [m*N + n])
    std::vector<std::complex<float>> h0;      // h0(k)
    std::vector<std::complex<float>> h0conj;  // conj(h0(-k))
    std::vector<float>               omega;   // dispersion w(k)

    // per-frame work + output buffers (size N*N)
    std::vector<std::complex<float>> hkt;     // time-evolved spectrum -> spatial
    std::vector<float>               height;  // vertical displacement grid

    // raylib mesh (one periodic patch), rebuilt once, updated each frame
    Mesh  mesh{};
    Model model{};
    bool  built=false;
};

// (re)build the Phillips spectrum from the current wind/amplitude parameters.
void oceanInit(Ocean& o);

// build the dynamic raylib mesh once (call after oceanInit, with a GL context).
void oceanBuildMesh(Ocean& o);

// advance the surface to time t and push new positions/normals to the GPU.
void oceanUpdate(Ocean& o, float t);

// periodic bilinear height sample at world (wx,wz) -- the buoyancy query.
float oceanSampleHeight(const Ocean& o, float wx, float wz);

void oceanUnload(Ocean& o);
