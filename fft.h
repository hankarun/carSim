// =============================================================================
//  fft.h -- tiny self-contained iterative radix-2 Cooley-Tukey FFT.
//  Header-only, no third-party dependencies.  Used by the FFT ocean (ocean.cpp).
//
//  fft1d(): in-place 1-D transform of a power-of-two length buffer.
//  fft2d(): 2-D transform of an NxN row-major buffer (FFT rows then columns).
//  Inverse transforms divide by N (1-D) / N*N (2-D) so ifft(fft(x)) == x.
// =============================================================================
#pragma once
#include <complex>
#include <vector>

namespace oc {

// in-place 1-D FFT of length n (must be a power of two).
inline void fft1d(std::complex<float>* a, int n, bool inverse){
    // bit-reversal permutation
    for(int i=1,j=0;i<n;++i){
        int bit=n>>1;
        for(;j&bit;bit>>=1) j^=bit;
        j^=bit;
        if(i<j) std::swap(a[i],a[j]);
    }
    const float TWO_PI=6.28318530717958647692f;
    for(int len=2;len<=n;len<<=1){
        float ang=TWO_PI/len*(inverse?1.0f:-1.0f);
        std::complex<float> wlen(std::cos(ang),std::sin(ang));
        for(int i=0;i<n;i+=len){
            std::complex<float> w(1.0f,0.0f);
            for(int k=0;k<len/2;++k){
                std::complex<float> u=a[i+k];
                std::complex<float> v=a[i+k+len/2]*w;
                a[i+k]      =u+v;
                a[i+k+len/2]=u-v;
                w*=wlen;
            }
        }
    }
    if(inverse) for(int i=0;i<n;++i) a[i]/=(float)n;
}

// in-place 2-D FFT of an N x N row-major grid (data[m*N + n]).
inline void fft2d(std::vector<std::complex<float>>& data, int N, bool inverse){
    std::vector<std::complex<float>> line(N);
    // transform each row
    for(int m=0;m<N;++m){
        for(int n=0;n<N;++n) line[n]=data[(size_t)m*N+n];
        fft1d(line.data(),N,inverse);
        for(int n=0;n<N;++n) data[(size_t)m*N+n]=line[n];
    }
    // transform each column
    for(int n=0;n<N;++n){
        for(int m=0;m<N;++m) line[m]=data[(size_t)m*N+n];
        fft1d(line.data(),N,inverse);
        for(int m=0;m<N;++m) data[(size_t)m*N+n]=line[m];
    }
}

} // namespace oc
