#pragma once
/**
 * \file NETGeographicLib/SphericalHarmonic.h
 * \brief Header for NETGeographicLib::SphericalHarmonic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    ref class CircularEngine;
    ref class SphericalCoefficients;
  /**
   * \brief .NET wrapper for GeographicLib::SphericalHarmonic.
   *
   * This class allows .NET applications to access GeographicLib::SphericalHarmonic.
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
   * - P<sub>\e nm</sub>(\e t) is the associated Legendre polynomial of degree
   *   \e n and order \e m.
   *
   * Two normalizations are supported for P<sub>\e nm</sub>
   * - fully normalized denoted by SphericalHarmonic::FULL.
   * - Schmidt semi-normalized denoted by SphericalHarmonic::SCHMIDT.
   *
   * Clenshaw summation is used for the sums over both \e n and \e m.  This
   * allows the computation to be carried out without the need for any
   * temporary arrays.  See GeographicLib::SphericalEngine.cpp for more
   * information on the implementation.
   *
   * References:
   * - C. W. Clenshaw, A note on the summation of Chebyshev series,
   *   %Math. Tables Aids Comput. 9(51), 118--120 (1955).
   * - R. E. Deakin, Derivatives of the earth's potentials, Geomatics
   *   Research Australasia 68, 31--60, (June 1998).
   * - W. A. Heiskanen and H. Moritz, Physical Geodesy, (Freeman, San
   *   Francisco, 1967).  (See Sec. 1-14, for a definition of Pbar.)
   * - S. A. Holmes and W. E. Featherstone, A unified approach to the Clenshaw
   *   summation and the recursive computation of very high degree and order
   *   normalised associated Legendre functions, J. Geodesy 76(5),
   *   279--299 (2002).
   * - C. C. Tscherning and K. Poder, Some geodetic applications of Clenshaw
   *   summation, Boll. Geod. Sci. Aff. 41(4), 349--375 (1982).
   *
   * C# Example:
   * \include example-SphericalHarmonic.cs
   * Managed C++ Example:
   * \include example-SphericalHarmonic.cpp
   * Visual Basic Example:
   * \include example-SphericalHarmonic.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * This class replaces the GeographicLib::SphericalHarmonic::operator() with
   * HarmonicSum.
   *
   * Coefficients returns a SphericalCoefficients object.
   *
   * The Normalization parameter in the constructors is passed in as an
   * enumeration rather than an unsigned.
   **********************************************************************/
    public ref class SphericalHarmonic
    {
        private:
        // a pointer to the unmanaged GeographicLib::SphericalHarmonic
        const GeographicLib::SphericalHarmonic* m_pSphericalHarmonic;
        // the finalizer frees the unmanaged memory when the object is destroyed.
        !SphericalHarmonic();
        // local containers for the cosine and sine coefficients.  The
        // GeographicLib::SphericalEngine::coeffs class uses a
        // std::vector::iterator to access these vectors.
        std::vector<double> *m_C, *m_S;
    public:
        /**
         * Supported normalizations for the associated Legendre polynomials.
         **********************************************************************/
        enum class Normalization {
          /**
           * Fully normalized associated Legendre polynomials.
           *
           * These are defined by <i>P</i><sub><i>nm</i></sub><sup>full</sup>(\e z)
           * = (&minus;1)<sup><i>m</i></sup> sqrt(\e k (2\e n + 1) (\e n &minus; \e
           * m)! / (\e n + \e m)!)
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
          FULL = GeographicLib::SphericalEngine::FULL,
          /**
           * Schmidt semi-normalized associated Legendre polynomials.
           *
           * These are defined by <i>P</i><sub><i>nm</i></sub><sup>schmidt</sup>(\e
           * z) = (&minus;1)<sup><i>m</i></sup> sqrt(\e k (\e n &minus; \e m)! /
           * (\e n + \e m)!)  <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z),
           * where <b>P</b><sub><i>n</i></sub><sup><i>m</i></sup>(\e z) is Ferrers
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
          SCHMIDT = GeographicLib::SphericalEngine::SCHMIDT,
        };

        /**
         * Constructor with a full set of coefficients specified.
         *
         * @param[in] C the coefficients \e C<sub>\e nm</sub>.
         * @param[in] S the coefficients \e S<sub>\e nm</sub>.
         * @param[in] N the maximum degree and order of the sum
         * @param[in] a the reference radius appearing in the definition of the
         *   sum.
         * @param[in] norm the normalization for the associated Legendre
         *   polynomials, either SphericalHarmonic::full (the default) or
         *   SphericalHarmonic::schmidt.
         * @exception GeographicErr if \e N does not satisfy \e N &ge; &minus;1.
         * @exception GeographicErr if \e C or \e S is not big enough to hold the
         *   coefficients.
         *
         * The coefficients \e C<sub>\e nm</sub> and \e S<sub>\e nm</sub> are
         * stored in the one-dimensional vectors \e C and \e S which must contain
         * (\e N + 1)(\e N + 2)/2 and N (\e N + 1)/2 elements, respectively, stored
         * in "column-major" order.  Thus for \e N = 3, the order would be:
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
        SphericalHarmonic(array<double>^ C,
                          array<double>^ S,
                          int N, double a, Normalization norm );

        /**
         * Constructor with a subset of coefficients specified.
         *
         * @param[in] C the coefficients \e C<sub>\e nm</sub>.
         * @param[in] S the coefficients \e S<sub>\e nm</sub>.
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
        SphericalHarmonic(array<double>^ C,
                          array<double>^ S,
                          int N, int nmx, int mmx,
                          double a, Normalization norm);

        /**
         * The destructor calls the finalizer
         **********************************************************************/
        ~SphericalHarmonic() { this->!SphericalHarmonic(); }

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
        double HarmonicSum(double x, double y, double z);

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
        double HarmonicSum(double x, double y, double z,
            [System::Runtime::InteropServices::Out] double% gradx,
            [System::Runtime::InteropServices::Out] double% grady,
            [System::Runtime::InteropServices::Out] double% gradz);

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
         * definition, i.e., &sum;<sub>n = 0..N</sub> &sum;<sub>m = 0..n</sub>
         * becomes &sum;<sub>m = 0..N</sub> &sum;<sub>n = m..N</sub>.
         * SphericalHarmonic::Circle performs the inner sum over degree \e n (which
         * entails about <i>N</i><sup>2</sup> operations).  Calling
         * CircularEngine::operator()() on the returned object performs the outer
         * sum over the order \e m (about \e N operations).
         **********************************************************************/
        CircularEngine^ Circle(double p, double z, bool gradp);

        /**
        * @return the zeroth SphericalCoefficients object.
        **********************************************************************/
        SphericalCoefficients^ Coefficients();
    };
} // namespace NETGeographicLib
