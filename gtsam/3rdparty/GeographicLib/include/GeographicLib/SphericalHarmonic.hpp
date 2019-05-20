/**
 * \file SphericalHarmonic.hpp
 * \brief Header for GeographicLib::SphericalHarmonic class
 *
 * Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_SPHERICALHARMONIC_HPP)
#define GEOGRAPHICLIB_SPHERICALHARMONIC_HPP 1

#include <vector>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/SphericalEngine.hpp>
#include <GeographicLib/CircularEngine.hpp>

namespace GeographicLib {

  /**
   * \brief Spherical harmonic series
   *
   * This class evaluates the spherical harmonic sum \verbatim
   V(x, y, z) = sum(n = 0..N)[ q^(n+1) * sum(m = 0..n)[
     (C[n,m] * cos(m*lambda) + S[n,m] * sin(m*lambda)) *
     P[n,m](cos(theta)) ] ]
   \endverbatim
   * where
   * - <i>p</i><sup>2</sup> = <i>x</i><sup>2</sup> + <i>y</i><sup>2</sup>,
   * - <i>r</i><sup>2</sup> = <i>p</i><sup>2</sup> + <i>z</i><sup>2</sup>,
   * - \e q = <i>a</i>/<i>r</i>,
   * - &theta; = atan2(\e p, \e z) = the spherical \e colatitude,
   * - &lambda; = atan2(\e y, \e x) = the longitude.
   * - P<sub><i>nm</i></sub>(\e t) is the associated Legendre polynomial of
   *   degree \e n and order \e m.
   *
   * Two normalizations are supported for P<sub><i>nm</i></sub>
   * - fully normalized denoted by SphericalHarmonic::FULL.
   * - Schmidt semi-normalized denoted by SphericalHarmonic::SCHMIDT.
   *
   * Clenshaw summation is used for the sums over both \e n and \e m.  This
   * allows the computation to be carried out without the need for any
   * temporary arrays.  See SphericalEngine.cpp for more information on the
   * implementation.
   *
   * References:
   * - C. W. Clenshaw,
   *   <a href="https://doi.org/10.1090/S0025-5718-1955-0071856-0">
   *   A note on the summation of Chebyshev series</a>,
   *   %Math. Tables Aids Comput. 9(51), 118--120 (1955).
   * - R. E. Deakin, Derivatives of the earth's potentials, Geomatics
   *   Research Australasia 68, 31--60, (June 1998).
   * - W. A. Heiskanen and H. Moritz, Physical Geodesy, (Freeman, San
   *   Francisco, 1967).  (See Sec. 1-14, for a definition of Pbar.)
   * - S. A. Holmes and W. E. Featherstone,
   *   <a href="https://doi.org/10.1007/s00190-002-0216-2">
   *   A unified approach to the Clenshaw summation and the recursive
   *   computation of very high degree and order normalised associated Legendre
   *   functions</a>, J. Geodesy 76(5), 279--299 (2002).
   * - C. C. Tscherning and K. Poder,
   *   <a href="http://cct.gfy.ku.dk/publ_cct/cct80.pdf">
   *   Some geodetic applications of Clenshaw summation</a>,
   *   Boll. Geod. Sci. Aff. 41(4), 349--375 (1982).
   *
   * Example of use:
   * \include example-SphericalHarmonic.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT SphericalHarmonic {
  public:
    /**
     * Supported normalizations for the associated Legendre polynomials.
     **********************************************************************/
    enum normalization {
      /**
       * Fully normalized associated Legendre polynomials.
       *
       * These are defined by
       * <i>P</i><sub><i>nm</i></sub><sup>full</sup>(\e z)
       * = (&minus;1)<sup><i>m</i></sup>
       * sqrt(\e k (2\e n + 1) (\e n &minus; \e m)! / (\e n + \e m)!)
       * <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z), where
       * <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z) is Ferrers
       * function (also known as the Legendre function on the cut or the
       * associated Legendre polynomial) http://dlmf.nist.gov/14.7.E10 and \e k
       * = 1 for \e m = 0 and \e k = 2 otherwise.
       *
       * The mean squared value of
       * <i>P</i><sub><i>nm</i></sub><sup>full</sup>(cos&theta;)
       * cos(<i>m</i>&lambda;) and
       * <i>P</i><sub><i>nm</i></sub><sup>full</sup>(cos&theta;)
       * sin(<i>m</i>&lambda;) over the sphere is 1.
       *
       * @hideinitializer
       **********************************************************************/
      FULL = SphericalEngine::FULL,
      /**
       * Schmidt semi-normalized associated Legendre polynomials.
       *
       * These are defined by
       * <i>P</i><sub><i>nm</i></sub><sup>schmidt</sup>(\e z)
       * = (&minus;1)<sup><i>m</i></sup>
       * sqrt(\e k (\e n &minus; \e m)! / (\e n + \e m)!)
       * <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z), where
       * <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z) is Ferrers
       * function (also known as the Legendre function on the cut or the
       * associated Legendre polynomial) http://dlmf.nist.gov/14.7.E10 and \e k
       * = 1 for \e m = 0 and \e k = 2 otherwise.
       *
       * The mean squared value of
       * <i>P</i><sub><i>nm</i></sub><sup>schmidt</sup>(cos&theta;)
       * cos(<i>m</i>&lambda;) and
       * <i>P</i><sub><i>nm</i></sub><sup>schmidt</sup>(cos&theta;)
       * sin(<i>m</i>&lambda;) over the sphere is 1/(2\e n + 1).
       *
       * @hideinitializer
       **********************************************************************/
      SCHMIDT = SphericalEngine::SCHMIDT,
    };

  private:
    typedef Math::real real;
    SphericalEngine::coeff _c[1];
    real _a;
    unsigned _norm;

  public:
    /**
     * Constructor with a full set of coefficients specified.
     *
     * @param[in] C the coefficients <i>C</i><sub><i>nm</i></sub>.
     * @param[in] S the coefficients <i>S</i><sub><i>nm</i></sub>.
     * @param[in] N the maximum degree and order of the sum
     * @param[in] a the reference radius appearing in the definition of the
     *   sum.
     * @param[in] norm the normalization for the associated Legendre
     *   polynomials, either SphericalHarmonic::FULL (the default) or
     *   SphericalHarmonic::SCHMIDT.
     * @exception GeographicErr if \e N does not satisfy \e N &ge; &minus;1.
     * @exception GeographicErr if \e C or \e S is not big enough to hold the
     *   coefficients.
     *
     * The coefficients <i>C</i><sub><i>nm</i></sub> and
     * <i>S</i><sub><i>nm</i></sub> are stored in the one-dimensional vectors
     * \e C and \e S which must contain (\e N + 1)(\e N + 2)/2 and \e N (\e N +
     * 1)/2 elements, respectively, stored in "column-major" order.  Thus for
     * \e N = 3, the order would be:
     * <i>C</i><sub>00</sub>,
     * <i>C</i><sub>10</sub>,
     * <i>C</i><sub>20</sub>,
     * <i>C</i><sub>30</sub>,
     * <i>C</i><sub>11</sub>,
     * <i>C</i><sub>21</sub>,
     * <i>C</i><sub>31</sub>,
     * <i>C</i><sub>22</sub>,
     * <i>C</i><sub>32</sub>,
     * <i>C</i><sub>33</sub>.
     * In general the (\e n,\e m) element is at index \e m \e N &minus; \e m
     * (\e m &minus; 1)/2 + \e n.  The layout of \e S is the same except that
     * the first column is omitted (since the \e m = 0 terms never contribute
     * to the sum) and the 0th element is <i>S</i><sub>11</sub>
     *
     * The class stores <i>pointers</i> to the first elements of \e C and \e S.
     * These arrays should not be altered or destroyed during the lifetime of a
     * SphericalHarmonic object.
     **********************************************************************/
    SphericalHarmonic(const std::vector<real>& C,
                      const std::vector<real>& S,
                      int N, real a, unsigned norm = FULL)
      : _a(a)
      , _norm(norm)
    { _c[0] = SphericalEngine::coeff(C, S, N); }

    /**
     * Constructor with a subset of coefficients specified.
     *
     * @param[in] C the coefficients <i>C</i><sub><i>nm</i></sub>.
     * @param[in] S the coefficients <i>S</i><sub><i>nm</i></sub>.
     * @param[in] N the degree used to determine the layout of \e C and \e S.
     * @param[in] nmx the maximum degree used in the sum.  The sum over \e n is
     *   from 0 thru \e nmx.
     * @param[in] mmx the maximum order used in the sum.  The sum over \e m is
     *   from 0 thru min(\e n, \e mmx).
     * @param[in] a the reference radius appearing in the definition of the
     *   sum.
     * @param[in] norm the normalization for the associated Legendre
     *   polynomials, either SphericalHarmonic::FULL (the default) or
     *   SphericalHarmonic::SCHMIDT.
     * @exception GeographicErr if \e N, \e nmx, and \e mmx do not satisfy
     *   \e N &ge; \e nmx &ge; \e mmx &ge; &minus;1.
     * @exception GeographicErr if \e C or \e S is not big enough to hold the
     *   coefficients.
     *
     * The class stores <i>pointers</i> to the first elements of \e C and \e S.
     * These arrays should not be altered or destroyed during the lifetime of a
     * SphericalHarmonic object.
     **********************************************************************/
    SphericalHarmonic(const std::vector<real>& C,
                      const std::vector<real>& S,
                      int N, int nmx, int mmx,
                      real a, unsigned norm = FULL)
      : _a(a)
      , _norm(norm)
    { _c[0] = SphericalEngine::coeff(C, S, N, nmx, mmx); }

    /**
     * A default constructor so that the object can be created when the
     * constructor for another object is initialized.  This default object can
     * then be reset with the default copy assignment operator.
     **********************************************************************/
    SphericalHarmonic() {}

    /**
     * Compute the spherical harmonic sum.
     *
     * @param[in] x cartesian coordinate.
     * @param[in] y cartesian coordinate.
     * @param[in] z cartesian coordinate.
     * @return \e V the spherical harmonic sum.
     *
     * This routine requires constant memory and thus never throws an
     * exception.
     **********************************************************************/
    Math::real operator()(real x, real y, real z) const {
      real f[] = {1};
      real v = 0;
      real dummy;
      switch (_norm) {
      case FULL:
        v = SphericalEngine::Value<false, SphericalEngine::FULL, 1>
          (_c, f, x, y, z, _a, dummy, dummy, dummy);
        break;
      case SCHMIDT:
        v = SphericalEngine::Value<false, SphericalEngine::SCHMIDT, 1>
          (_c, f, x, y, z, _a, dummy, dummy, dummy);
        break;
      }
      return v;
    }

    /**
     * Compute a spherical harmonic sum and its gradient.
     *
     * @param[in] x cartesian coordinate.
     * @param[in] y cartesian coordinate.
     * @param[in] z cartesian coordinate.
     * @param[out] gradx \e x component of the gradient
     * @param[out] grady \e y component of the gradient
     * @param[out] gradz \e z component of the gradient
     * @return \e V the spherical harmonic sum.
     *
     * This is the same as the previous function, except that the components of
     * the gradients of the sum in the \e x, \e y, and \e z directions are
     * computed.  This routine requires constant memory and thus never throws
     * an exception.
     **********************************************************************/
    Math::real operator()(real x, real y, real z,
                          real& gradx, real& grady, real& gradz) const {
      real f[] = {1};
      real v = 0;
      switch (_norm) {
      case FULL:
        v = SphericalEngine::Value<true, SphericalEngine::FULL, 1>
          (_c, f, x, y, z, _a, gradx, grady, gradz);
        break;
      case SCHMIDT:
        v = SphericalEngine::Value<true, SphericalEngine::SCHMIDT, 1>
          (_c, f, x, y, z, _a, gradx, grady, gradz);
        break;
      }
      return v;
    }

    /**
     * Create a CircularEngine to allow the efficient evaluation of several
     * points on a circle of latitude.
     *
     * @param[in] p the radius of the circle.
     * @param[in] z the height of the circle above the equatorial plane.
     * @param[in] gradp if true the returned object will be able to compute the
     *   gradient of the sum.
     * @exception std::bad_alloc if the memory for the CircularEngine can't be
     *   allocated.
     * @return the CircularEngine object.
     *
     * SphericalHarmonic::operator()() exchanges the order of the sums in the
     * definition, i.e., &sum;<sub><i>n</i> = 0..<i>N</i></sub>
     * &sum;<sub><i>m</i> = 0..<i>n</i></sub> becomes &sum;<sub><i>m</i> =
     * 0..<i>N</i></sub> &sum;<sub><i>n</i> = <i>m</i>..<i>N</i></sub>.
     * SphericalHarmonic::Circle performs the inner sum over degree \e n (which
     * entails about <i>N</i><sup>2</sup> operations).  Calling
     * CircularEngine::operator()() on the returned object performs the outer
     * sum over the order \e m (about \e N operations).
     *
     * Here's an example of computing the spherical sum at a sequence of
     * longitudes without using a CircularEngine object \code
     SphericalHarmonic h(...);     // Create the SphericalHarmonic object
     double r = 2, lat = 33, lon0 = 44, dlon = 0.01;
     double
       phi = lat * Math::degree<double>(),
       z = r * sin(phi), p = r * cos(phi);
     for (int i = 0; i <= 100; ++i) {
       real
         lon = lon0 + i * dlon,
         lam = lon * Math::degree<double>();
       std::cout << lon << " " << h(p * cos(lam), p * sin(lam), z) << "\n";
     }
     \endcode
     * Here is the same calculation done using a CircularEngine object.  This
     * will be about <i>N</i>/2 times faster. \code
     SphericalHarmonic h(...);     // Create the SphericalHarmonic object
     double r = 2, lat = 33, lon0 = 44, dlon = 0.01;
     double
       phi = lat * Math::degree<double>(),
       z = r * sin(phi), p = r * cos(phi);
     CircularEngine c(h(p, z, false)); // Create the CircularEngine object
     for (int i = 0; i <= 100; ++i) {
       real
         lon = lon0 + i * dlon;
       std::cout << lon << " " << c(lon) << "\n";
     }
     \endcode
     **********************************************************************/
    CircularEngine Circle(real p, real z, bool gradp) const {
      real f[] = {1};
      switch (_norm) {
      case FULL:
        return gradp ?
          SphericalEngine::Circle<true, SphericalEngine::FULL, 1>
          (_c, f, p, z, _a) :
          SphericalEngine::Circle<false, SphericalEngine::FULL, 1>
          (_c, f, p, z, _a);
        break;
      case SCHMIDT:
      default:                  // To avoid compiler warnings
        return gradp ?
          SphericalEngine::Circle<true, SphericalEngine::SCHMIDT, 1>
          (_c, f, p, z, _a) :
          SphericalEngine::Circle<false, SphericalEngine::SCHMIDT, 1>
          (_c, f, p, z, _a);
        break;
      }
    }

    /**
     * @return the zeroth SphericalEngine::coeff object.
     **********************************************************************/
    const SphericalEngine::coeff& Coefficients() const
    { return _c[0]; }
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_SPHERICALHARMONIC_HPP
