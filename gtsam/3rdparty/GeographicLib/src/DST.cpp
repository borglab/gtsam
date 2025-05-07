/**
 * \file DST.cpp
 * \brief Implementation for GeographicLib::DST class
 *
 * Copyright (c) Charles Karney (2022) <karney@alum.mit.edu> and licensed under
 * the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/DST.hpp>
#include <vector>

#include "kissfft.hh"

namespace GeographicLib {

  using namespace std;

  DST::DST(int N)
    : _nN(N < 0 ? 0 : N)
    , _fft(make_shared<fft_t>(fft_t(2 * _nN, false)))
  {}

  void DST::reset(int N) {
    N = N < 0 ? 0 : N;
    if (N == _nN) return;
    _nN = N;
    _fft->assign(2 * _nN, false);
  }

  void DST::fft_transform(real data[], real F[], bool centerp) const {
    // Implement DST-III (centerp = false) or DST-IV (centerp = true).

    // Elements (0,N], resp. [0,N), of data should be set on input for centerp
    // = false, resp. true.  F must have a size of at least N and on output
    // elements [0,N) of F contain the transform.
    if (_nN == 0) return;
    if (centerp) {
      for (int i = 0; i < _nN; ++i) {
        data[_nN+i] = data[_nN-1-i];
        data[2*_nN+i] = -data[i];
        data[3*_nN+i] = -data[_nN-1-i];
      }
    } else {
      data[0] = 0;              // set [0]
      for (int i = 1; i < _nN; ++i)
        data[_nN+i] = data[_nN-i]; // set [N+1,2*N-1]
      for (int i = 0; i < 2*_nN; ++i)
        data[2*_nN+i] = -data[i]; // [2*N, 4*N-1]
    }
    vector<complex<real>> ctemp(2*_nN);
    _fft->transform_real(data, ctemp.data());
    if (centerp) {
      real d = -Math::pi()/(4*_nN);
      for (int i = 0, j = 1; i < _nN; ++i, j+=2)
        ctemp[j] *= exp(complex<real>(0, j*d));
    }
    for (int i = 0, j = 1; i < _nN; ++i, j+=2) {
      F[i] = -ctemp[j].imag() / (2*_nN);
    }
  }

  void DST::fft_transform2(real data[], real F[]) const {
    // Elements [0,N), of data should be set to the N grid center values and F
    // should have size of at least 2*N.  On input elements [0,N) of F contain
    // the size N transform; on output elements [0,2*N) of F contain the size
    // 2*N transform.
    fft_transform(data, F+_nN, true);
    // Copy DST-IV order N tx to [0,N) elements of data
    for (int i = 0; i < _nN; ++i) data[i] = F[i+_nN];
    for (int i = _nN; i < 2*_nN; ++i)
      // (DST-IV order N - DST-III order N) / 2
      F[i] = (data[2*_nN-1-i] - F[2*_nN-1-i])/2;
    for (int i = 0; i < _nN; ++i)
      // (DST-IV order N + DST-III order N) / 2
      F[i] = (data[i] + F[i])/2;
  }

  void DST::transform(function<real(real)> f, real F[]) const {
    vector<real> data(4 * _nN);
    real d = Math::pi()/(2 * _nN);
    for (int i = 1; i <= _nN; ++i)
      data[i] = f( i * d );
    fft_transform(data.data(), F, false);
  }

  void DST::refine(function<real(real)> f, real F[]) const {
    vector<real> data(4 * _nN);
    real d = Math::pi()/(4 * _nN);
    for (int i = 0; i < _nN; ++i)
      data[i] = f( (2*i + 1) * d );
    fft_transform2(data.data(), F);
  }

  Math::real DST::eval(real sinx, real cosx, const real F[], int N) {
    // Evaluate
    // y = sum(F[i] * sin((2*i+1) * x), i, 0, N-1)
    // using Clenshaw summation.
    // Approx operation count = (N + 5) mult and (2 * N + 2) add
    real
      ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
      y0 = N & 1 ? F[--N] : 0, y1 = 0;          // accumulators for sum
    // Now N is even
    while (N > 0) {
      // Unroll loop x 2, so accumulators return to their original role
      y1 = ar * y0 - y1 + F[--N];
      y0 = ar * y1 - y0 + F[--N];
    }
    return sinx * (y0 + y1);    // sin(x) * (y0 + y1)
  }

  Math::real DST::integral(real sinx, real cosx, const real F[], int N) {
    // Evaluate
    // y = -sum(F[i]/(2*i+1) * cos((2*i+1) * x), i, 0, N-1)
    // using Clenshaw summation.
    // Approx operation count = (N + 5) mult and (2 * N + 2) add
    real
      ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
      y0 = 0, y1 = 0;                         // accumulators for sum
    for (--N; N >= 0; --N) {
      real t = ar * y0 - y1 + F[N]/(2*N+1);
      y1 = y0; y0 = t;
    }
    return cosx * (y1 - y0);    // cos(x) * (y1 - y0)
  }

  Math::real DST::integral(real sinx, real cosx, real siny, real cosy,
                           const real F[], int N) {
    // return integral(siny, cosy, F, N) - integral(sinx, cosx, F, N);
    real
      // 2*cos(y-x)*cos(y+x) -> 2 * cos(2 * x)
      ac = +2 * (cosy * cosx + siny * sinx) * (cosy * cosx - siny * sinx),
      // -2*sin(y-x)*sin(y+x) -> 0
      as = -2 * (siny * cosx - cosy * sinx) * (siny * cosx + cosy * sinx),
      y0 = 0, y1 = 0, z0 = 0, z1 = 0; // accumulators for sum
    for (--N; N >= 0; --N) {
      real
        ty = ac * y0 + as * z0 - y1 + F[N]/(2*N+1),
        tz = as * y0 + ac * z0 - z1;
      y1 = y0; y0 = ty;
      z1 = z0; z0 = tz;
    }
    // B[0] - B[1] = [y0-y1, z0-z1]
    // F[0] = [cosy + cosx, cosy - cosx] -> [2 * cosx, 0]
    // (B[0] - B[1]) . F[0]
    // = [(y0 - y1) * (cosy + cosx) + (z0 - z1) * (cosy - cosx),
    //    (y0 - y1) * (cosy - cosx) + (z0 - z1) * (cosy + cosx),
    // return -(2nd element)
    return (y1 - y0) * (cosy - cosx) + (z1 - z0) * (cosy + cosx);
  }

} // namespace GeographicLib
