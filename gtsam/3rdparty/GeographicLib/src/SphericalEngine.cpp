/**
 * \file SphericalEngine.cpp
 * \brief Implementation for GeographicLib::SphericalEngine class
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * The general sum is\verbatim
 V(r, theta, lambda) = sum(n = 0..N) sum(m = 0..n)
   q^(n+1) * (C[n,m] * cos(m*lambda) + S[n,m] * sin(m*lambda)) * P[n,m](t)
\endverbatim
 * where <tt>t = cos(theta)</tt>, <tt>q = a/r</tt>.  In addition write <tt>u =
 * sin(theta)</tt>.
 *
 * <tt>P[n,m]</tt> is a normalized associated Legendre function of degree
 * <tt>n</tt> and order <tt>m</tt>.  Here the formulas are given for full
 * normalized functions (usually denoted <tt>Pbar</tt>).
 *
 * Rewrite outer sum\verbatim
 V(r, theta, lambda) = sum(m = 0..N) * P[m,m](t) * q^(m+1) *
    [Sc[m] * cos(m*lambda) + Ss[m] * sin(m*lambda)]
\endverbatim
 * where the inner sums are\verbatim
   Sc[m] = sum(n = m..N) q^(n-m) * C[n,m] * P[n,m](t)/P[m,m](t)
   Ss[m] = sum(n = m..N) q^(n-m) * S[n,m] * P[n,m](t)/P[m,m](t)
\endverbatim
 * Evaluate sums via Clenshaw method.  The overall framework is similar to
 * Deakin with the following changes:
 * - Clenshaw summation is used to roll the computation of
 *   <tt>cos(m*lambda)</tt> and <tt>sin(m*lambda)</tt> into the evaluation of
 *   the outer sum (rather than independently computing an array of these
 *   trigonometric terms).
 * - Scale the coefficients to guard against overflow when <tt>N</tt> is large.
 * .
 * For the general framework of Clenshaw, see
 * http://mathworld.wolfram.com/ClenshawRecurrenceFormula.html
 *
 * Let\verbatim
    S = sum(k = 0..N) c[k] * F[k](x)
    F[n+1](x) = alpha[n](x) * F[n](x) + beta[n](x) * F[n-1](x)
\endverbatim
 * Evaluate <tt>S</tt> with\verbatim
    y[N+2] = y[N+1] = 0
    y[k] = alpha[k] * y[k+1] + beta[k+1] * y[k+2] + c[k]
    S = c[0] * F[0] + y[1] * F[1] + beta[1] * F[0] * y[2]
\endverbatim
 * \e IF <tt>F[0](x) = 1</tt> and <tt>beta(0,x) = 0</tt>, then <tt>F[1](x) =
 * alpha(0,x)</tt> and we can continue the recursion for <tt>y[k]</tt> until
 * <tt>y[0]</tt>, giving\verbatim
    S = y[0]
\endverbatim
 *
 * Evaluating the inner sum\verbatim
 l = n-m; n = l+m
 Sc[m] = sum(l = 0..N-m) C[l+m,m] * q^l * P[l+m,m](t)/P[m,m](t)
 F[l] = q^l * P[l+m,m](t)/P[m,m](t)
\endverbatim
 * Holmes + Featherstone, Eq. (11), give\verbatim
   P[n,m] = sqrt((2*n-1)*(2*n+1)/((n-m)*(n+m))) * t * P[n-1,m] -
            sqrt((2*n+1)*(n+m-1)*(n-m-1)/((n-m)*(n+m)*(2*n-3))) * P[n-2,m]
\endverbatim
 * thus\verbatim
   alpha[l] = t * q * sqrt(((2*n+1)*(2*n+3))/
                           ((n-m+1)*(n+m+1)))
   beta[l+1] = - q^2 * sqrt(((n-m+1)*(n+m+1)*(2*n+5))/
                            ((n-m+2)*(n+m+2)*(2*n+1)))
\endverbatim
 * In this case, <tt>F[0] = 1</tt> and <tt>beta[0] = 0</tt>, so the <tt>Sc[m]
 * = y[0]</tt>.
 *
 * Evaluating the outer sum\verbatim
 V = sum(m = 0..N) Sc[m] * q^(m+1) * cos(m*lambda) * P[m,m](t)
   + sum(m = 0..N) Ss[m] * q^(m+1) * cos(m*lambda) * P[m,m](t)
 F[m] = q^(m+1) * cos(m*lambda) * P[m,m](t) [or sin(m*lambda)]
\endverbatim
 * Holmes + Featherstone, Eq. (13), give\verbatim
   P[m,m] = u * sqrt((2*m+1)/((m>1?2:1)*m)) * P[m-1,m-1]
\endverbatim
 * also, we have\verbatim
   cos((m+1)*lambda) = 2*cos(lambda)*cos(m*lambda) - cos((m-1)*lambda)
\endverbatim
 * thus\verbatim
   alpha[m] = 2*cos(lambda) * sqrt((2*m+3)/(2*(m+1))) * u * q
            =   cos(lambda) * sqrt( 2*(2*m+3)/(m+1) ) * u * q
   beta[m+1] = -sqrt((2*m+3)*(2*m+5)/(4*(m+1)*(m+2))) * u^2 * q^2
               * (m == 0 ? sqrt(2) : 1)
\endverbatim
 * Thus\verbatim
 F[0] = q                                [or 0]
 F[1] = cos(lambda) * sqrt(3) * u * q^2  [or sin(lambda)]
 beta[1] = - sqrt(15/4) * u^2 * q^2
\endverbatim
 *
 * Here is how the various components of the gradient are computed
 *
 * Differentiate wrt <tt>r</tt>\verbatim
   d q^(n+1) / dr = (-1/r) * (n+1) * q^(n+1)
\endverbatim
 * so multiply <tt>C[n,m]</tt> by <tt>n+1</tt> in inner sum and multiply the
 * sum by <tt>-1/r</tt>.
 *
 * Differentiate wrt <tt>lambda</tt>\verbatim
   d cos(m*lambda) = -m * sin(m*lambda)
   d sin(m*lambda) =  m * cos(m*lambda)
\endverbatim
 * so multiply terms by <tt>m</tt> in outer sum and swap sine and cosine
 * variables.
 *
 * Differentiate wrt <tt>theta</tt>\verbatim
  dV/dtheta = V' = -u * dV/dt = -u * V'
\endverbatim
 * here <tt>'</tt> denotes differentiation wrt <tt>theta</tt>.\verbatim
   d/dtheta (Sc[m] * P[m,m](t)) = Sc'[m] * P[m,m](t) + Sc[m] * P'[m,m](t)
\endverbatim
 * Now <tt>P[m,m](t) = const * u^m</tt>, so <tt>P'[m,m](t) = m * t/u *
 * P[m,m](t)</tt>, thus\verbatim
   d/dtheta (Sc[m] * P[m,m](t)) = (Sc'[m] + m * t/u * Sc[m]) * P[m,m](t)
\endverbatim
 * Clenshaw recursion for <tt>Sc[m]</tt> reads\verbatim
    y[k] = alpha[k] * y[k+1] + beta[k+1] * y[k+2] + c[k]
\endverbatim
 * Substituting <tt>alpha[k] = const * t</tt>, <tt>alpha'[k] = -u/t *
 * alpha[k]</tt>, <tt>beta'[k] = c'[k] = 0</tt> gives\verbatim
    y'[k] = alpha[k] * y'[k+1] + beta[k+1] * y'[k+2] - u/t * alpha[k] * y[k+1]
\endverbatim
 *
 * Finally, given the derivatives of <tt>V</tt>, we can compute the components
 * of the gradient in spherical coordinates and transform the result into
 * cartesian coordinates.
 **********************************************************************/

#include <GeographicLib/SphericalEngine.hpp>
#include <GeographicLib/CircularEngine.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

namespace GeographicLib {

  using namespace std;

  vector<Math::real>& SphericalEngine::sqrttable() {
    static vector<real> sqrttable(0);
    return sqrttable;
  }

  template<bool gradp, SphericalEngine::normalization norm, int L>
  Math::real SphericalEngine::Value(const coeff c[], const real f[],
                                    real x, real y, real z, real a,
                                    real& gradx, real& grady, real& gradz)
    {
    GEOGRAPHICLIB_STATIC_ASSERT(L > 0, "L must be positive");
    GEOGRAPHICLIB_STATIC_ASSERT(norm == FULL || norm == SCHMIDT,
                                "Unknown normalization");
    int N = c[0].nmx(), M = c[0].mmx();

    real
      p = Math::hypot(x, y),
      cl = p != 0 ? x / p : 1,  // cos(lambda); at pole, pick lambda = 0
      sl = p != 0 ? y / p : 0,  // sin(lambda)
      r = Math::hypot(z, p),
      t = r != 0 ? z / r : 0,   // cos(theta); at origin, pick theta = pi/2
      u = r != 0 ? max(p / r, eps()) : 1, // sin(theta); but avoid the pole
      q = a / r;
    real
      q2 = Math::sq(q),
      uq = u * q,
      uq2 = Math::sq(uq),
      tu = t / u;
    // Initialize outer sum
    real vc  = 0, vc2  = 0, vs  = 0, vs2  = 0;   // v [N + 1], v [N + 2]
    // vr, vt, vl and similar w variable accumulate the sums for the
    // derivatives wrt r, theta, and lambda, respectively.
    real vrc = 0, vrc2 = 0, vrs = 0, vrs2 = 0;   // vr[N + 1], vr[N + 2]
    real vtc = 0, vtc2 = 0, vts = 0, vts2 = 0;   // vt[N + 1], vt[N + 2]
    real vlc = 0, vlc2 = 0, vls = 0, vls2 = 0;   // vl[N + 1], vl[N + 2]
    int k[L];
    const vector<real>& root( sqrttable() );
    for (int m = M; m >= 0; --m) {   // m = M .. 0
      // Initialize inner sum
      real
        wc  = 0, wc2  = 0, ws  = 0, ws2  = 0, // w [N - m + 1], w [N - m + 2]
        wrc = 0, wrc2 = 0, wrs = 0, wrs2 = 0, // wr[N - m + 1], wr[N - m + 2]
        wtc = 0, wtc2 = 0, wts = 0, wts2 = 0; // wt[N - m + 1], wt[N - m + 2]
      for (int l = 0; l < L; ++l)
        k[l] = c[l].index(N, m) + 1;
      for (int n = N; n >= m; --n) {             // n = N .. m; l = N - m .. 0
        real w, A, Ax, B, R;    // alpha[l], beta[l + 1]
        switch (norm) {
        case FULL:
          w = root[2 * n + 1] / (root[n - m + 1] * root[n + m + 1]);
          Ax = q * w * root[2 * n + 3];
          A = t * Ax;
          B = - q2 * root[2 * n + 5] /
            (w * root[n - m + 2] * root[n + m + 2]);
          break;
        case SCHMIDT:
          w = root[n - m + 1] * root[n + m + 1];
          Ax = q * (2 * n + 1) / w;
          A = t * Ax;
          B = - q2 * w / (root[n - m + 2] * root[n + m + 2]);
          break;
        default: break;       // To suppress warning message from Visual Studio
        }
        R = c[0].Cv(--k[0]);
        for (int l = 1; l < L; ++l)
          R += c[l].Cv(--k[l], n, m, f[l]);
        R *= scale();
        w = A * wc + B * wc2 + R; wc2 = wc; wc = w;
        if (gradp) {
          w = A * wrc + B * wrc2 + (n + 1) * R; wrc2 = wrc; wrc = w;
          w = A * wtc + B * wtc2 -  u*Ax * wc2; wtc2 = wtc; wtc = w;
        }
        if (m) {
          R = c[0].Sv(k[0]);
          for (int l = 1; l < L; ++l)
            R += c[l].Sv(k[l], n, m, f[l]);
          R *= scale();
          w = A * ws + B * ws2 + R; ws2 = ws; ws = w;
          if (gradp) {
            w = A * wrs + B * wrs2 + (n + 1) * R; wrs2 = wrs; wrs = w;
            w = A * wts + B * wts2 -  u*Ax * ws2; wts2 = wts; wts = w;
          }
        }
      }
      // Now Sc[m] = wc, Ss[m] = ws
      // Sc'[m] = wtc, Ss'[m] = wtc
      if (m) {
        real v, A, B;           // alpha[m], beta[m + 1]
        switch (norm) {
        case FULL:
          v = root[2] * root[2 * m + 3] / root[m + 1];
          A = cl * v * uq;
          B = - v * root[2 * m + 5] / (root[8] * root[m + 2]) * uq2;
          break;
        case SCHMIDT:
          v = root[2] * root[2 * m + 1] / root[m + 1];
          A = cl * v * uq;
          B = - v * root[2 * m + 3] / (root[8] * root[m + 2]) * uq2;
          break;
        default: break;       // To suppress warning message from Visual Studio
        }
        v = A * vc  + B * vc2  +  wc ; vc2  = vc ; vc  = v;
        v = A * vs  + B * vs2  +  ws ; vs2  = vs ; vs  = v;
        if (gradp) {
          // Include the terms Sc[m] * P'[m,m](t) and Ss[m] * P'[m,m](t)
          wtc += m * tu * wc; wts += m * tu * ws;
          v = A * vrc + B * vrc2 +  wrc; vrc2 = vrc; vrc = v;
          v = A * vrs + B * vrs2 +  wrs; vrs2 = vrs; vrs = v;
          v = A * vtc + B * vtc2 +  wtc; vtc2 = vtc; vtc = v;
          v = A * vts + B * vts2 +  wts; vts2 = vts; vts = v;
          v = A * vlc + B * vlc2 + m*ws; vlc2 = vlc; vlc = v;
          v = A * vls + B * vls2 - m*wc; vls2 = vls; vls = v;
        }
      } else {
        real A, B, qs;
        switch (norm) {
        case FULL:
          A = root[3] * uq;       // F[1]/(q*cl) or F[1]/(q*sl)
          B = - root[15]/2 * uq2; // beta[1]/q
          break;
        case SCHMIDT:
          A = uq;
          B = - root[3]/2 * uq2;
          break;
        default: break;       // To suppress warning message from Visual Studio
        }
        qs = q / scale();
        vc = qs * (wc + A * (cl * vc + sl * vs ) + B * vc2);
        if (gradp) {
          qs /= r;
          // The components of the gradient in spherical coordinates are
          // r: dV/dr
          // theta: 1/r * dV/dtheta
          // lambda: 1/(r*u) * dV/dlambda
          vrc =   - qs * (wrc + A * (cl * vrc + sl * vrs) + B * vrc2);
          vtc =     qs * (wtc + A * (cl * vtc + sl * vts) + B * vtc2);
          vlc = qs / u * (      A * (cl * vlc + sl * vls) + B * vlc2);
        }
      }
    }

    if (gradp) {
      // Rotate into cartesian (geocentric) coordinates
      gradx = cl * (u * vrc + t * vtc) - sl * vlc;
      grady = sl * (u * vrc + t * vtc) + cl * vlc;
      gradz =       t * vrc - u * vtc            ;
    }
    return vc;
  }

  template<bool gradp, SphericalEngine::normalization norm, int L>
  CircularEngine SphericalEngine::Circle(const coeff c[], const real f[],
                                         real p, real z, real a) {

    GEOGRAPHICLIB_STATIC_ASSERT(L > 0, "L must be positive");
    GEOGRAPHICLIB_STATIC_ASSERT(norm == FULL || norm == SCHMIDT,
                                "Unknown normalization");
    int N = c[0].nmx(), M = c[0].mmx();

    real
      r = Math::hypot(z, p),
      t = r != 0 ? z / r : 0,   // cos(theta); at origin, pick theta = pi/2
      u = r != 0 ? max(p / r, eps()) : 1, // sin(theta); but avoid the pole
      q = a / r;
    real
      q2 = Math::sq(q),
      tu = t / u;
    CircularEngine circ(M, gradp, norm, a, r, u, t);
    int k[L];
    const vector<real>& root( sqrttable() );
    for (int m = M; m >= 0; --m) {   // m = M .. 0
      // Initialize inner sum
      real
        wc  = 0, wc2  = 0, ws  = 0, ws2  = 0, // w [N - m + 1], w [N - m + 2]
        wrc = 0, wrc2 = 0, wrs = 0, wrs2 = 0, // wr[N - m + 1], wr[N - m + 2]
        wtc = 0, wtc2 = 0, wts = 0, wts2 = 0; // wt[N - m + 1], wt[N - m + 2]
      for (int l = 0; l < L; ++l)
        k[l] = c[l].index(N, m) + 1;
      for (int n = N; n >= m; --n) {             // n = N .. m; l = N - m .. 0
        real w, A, Ax, B, R;    // alpha[l], beta[l + 1]
        switch (norm) {
        case FULL:
          w = root[2 * n + 1] / (root[n - m + 1] * root[n + m + 1]);
          Ax = q * w * root[2 * n + 3];
          A = t * Ax;
          B = - q2 * root[2 * n + 5] /
            (w * root[n - m + 2] * root[n + m + 2]);
          break;
        case SCHMIDT:
          w = root[n - m + 1] * root[n + m + 1];
          Ax = q * (2 * n + 1) / w;
          A = t * Ax;
          B = - q2 * w / (root[n - m + 2] * root[n + m + 2]);
          break;
        default: break;       // To suppress warning message from Visual Studio
        }
        R = c[0].Cv(--k[0]);
        for (int l = 1; l < L; ++l)
          R += c[l].Cv(--k[l], n, m, f[l]);
        R *= scale();
        w = A * wc + B * wc2 + R; wc2 = wc; wc = w;
        if (gradp) {
          w = A * wrc + B * wrc2 + (n + 1) * R; wrc2 = wrc; wrc = w;
          w = A * wtc + B * wtc2 -  u*Ax * wc2; wtc2 = wtc; wtc = w;
        }
        if (m) {
          R = c[0].Sv(k[0]);
          for (int l = 1; l < L; ++l)
            R += c[l].Sv(k[l], n, m, f[l]);
          R *= scale();
          w = A * ws + B * ws2 + R; ws2 = ws; ws = w;
          if (gradp) {
            w = A * wrs + B * wrs2 + (n + 1) * R; wrs2 = wrs; wrs = w;
            w = A * wts + B * wts2 -  u*Ax * ws2; wts2 = wts; wts = w;
          }
        }
      }
      if (!gradp)
        circ.SetCoeff(m, wc, ws);
      else {
        // Include the terms Sc[m] * P'[m,m](t) and  Ss[m] * P'[m,m](t)
        wtc += m * tu * wc; wts += m * tu * ws;
        circ.SetCoeff(m, wc, ws, wrc, wrs, wtc, wts);
      }
    }

    return circ;
  }

  void SphericalEngine::RootTable(int N) {
    // Need square roots up to max(2 * N + 5, 15).
    vector<real>& root( sqrttable() );
    int L = max(2 * N + 5, 15) + 1, oldL = int(root.size());
    if (oldL >= L)
      return;
    root.resize(L);
    for (int l = oldL; l < L; ++l)
      root[l] = sqrt(real(l));
  }

  void SphericalEngine::coeff::readcoeffs(std::istream& stream, int& N, int& M,
                                          std::vector<real>& C,
                                          std::vector<real>& S) {
    int nm[2];
    Utility::readarray<int, int, false>(stream, nm, 2);
    N = nm[0]; M = nm[1];
    if (!(N >= M && M >= -1 && N * M >= 0))
      // The last condition is that M = -1 implies N = -1 and vice versa.
      throw GeographicErr("Bad degree and order " +
                          Utility::str(N) + " " + Utility::str(M));
    C.resize(SphericalEngine::coeff::Csize(N, M));
    Utility::readarray<double, real, false>(stream, C);
    S.resize(SphericalEngine::coeff::Ssize(N, M));
    Utility::readarray<double, real, false>(stream, S);
    return;
  }

  /// \cond SKIP
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::FULL, 1>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::FULL, 1>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::SCHMIDT, 1>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::SCHMIDT, 1>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);

  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::FULL, 2>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::FULL, 2>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::SCHMIDT, 2>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::SCHMIDT, 2>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);

  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::FULL, 3>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::FULL, 3>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<true, SphericalEngine::SCHMIDT, 3>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);
  template Math::real GEOGRAPHICLIB_EXPORT
  SphericalEngine::Value<false, SphericalEngine::SCHMIDT, 3>
  (const coeff[], const real[], real, real, real, real, real&, real&, real&);

  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::FULL, 1>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::FULL, 1>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::SCHMIDT, 1>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::SCHMIDT, 1>
  (const coeff[], const real[], real, real, real);

  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::FULL, 2>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::FULL, 2>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::SCHMIDT, 2>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::SCHMIDT, 2>
  (const coeff[], const real[], real, real, real);

  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::FULL, 3>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::FULL, 3>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<true, SphericalEngine::SCHMIDT, 3>
  (const coeff[], const real[], real, real, real);
  template CircularEngine GEOGRAPHICLIB_EXPORT
  SphericalEngine::Circle<false, SphericalEngine::SCHMIDT, 3>
  (const coeff[], const real[], real, real, real);
  /// \endcond

} // namespace GeographicLib
