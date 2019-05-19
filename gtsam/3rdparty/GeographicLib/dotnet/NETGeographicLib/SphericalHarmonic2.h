#pragma once
/**
 * \file NETGeographicLib/SphericalHarmonic2.h
 * \brief Header for NETGeographicLib::SphericalHarmonic2 class
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
   * \brief .NET wrapper for GeographicLib::SphericalHarmonic2.
   *
   * This class allows .NET applications to access GeographicLib::SphericalHarmonic2.
   *
   * This class is similar to SphericalHarmonic, except that the coefficients
   * \e C<sub>\e nm</sub> are replaced by \e C<sub>\e nm</sub> + \e tau'
   * C'<sub>\e nm</sub> + \e tau'' C''<sub>\e nm</sub> (and similarly for \e
   * S<sub>\e nm</sub>).
   *
   * C# Example:
   * \include example-SphericalHarmonic2.cs
   * Managed C++ Example:
   * \include example-SphericalHarmonic2.cpp
   * Visual Basic Example:
   * \include example-SphericalHarmonic2.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * This class replaces the () operator with HarmonicSum().
   *
   * Coefficients, Coefficients1, and Coefficients2 return a SphericalCoefficients
   * object.
   **********************************************************************/
    public ref class SphericalHarmonic2
    {
        private:
        // pointer to the unmanaged GeographicLib::SphericalHarmonic2
        const GeographicLib::SphericalHarmonic2* m_pSphericalHarmonic2;
        // the finalizer destroys the unmanaged memory when the object is destroyed.
        !SphericalHarmonic2(void);
        // the number of coefficient vectors.
        static const int m_numCoeffVectors = 3;
        // local containers for the cosine and sine coefficients.  The
        // GeographicLib::SphericalEngine::coeffs class uses a
        // std::vector::iterator to access these vectors.
        std::vector<double> **m_C, **m_S;
    public:
        /**
         * Supported normalizations for associate Legendre polynomials.
         **********************************************************************/
        enum class Normalization {
          /**
           * Fully normalized associated Legendre polynomials.  See
           * SphericalHarmonic::FULL for documentation.
           *
           * @hideinitializer
           **********************************************************************/
          FULL = GeographicLib::SphericalEngine::FULL,
          /**
           * Schmidt semi-normalized associated Legendre polynomials.  See
           * SphericalHarmonic::SCHMIDT for documentation.
           *
           * @hideinitializer
           **********************************************************************/
          SCHMIDT = GeographicLib::SphericalEngine::SCHMIDT
        };

        /**
         * Constructor with a full set of coefficients specified.
         *
         * @param[in] C the coefficients \e C<sub>\e nm</sub>.
         * @param[in] S the coefficients \e S<sub>\e nm</sub>.
         * @param[in] N the maximum degree and order of the sum
         * @param[in] C1 the coefficients \e C'<sub>\e nm</sub>.
         * @param[in] S1 the coefficients \e S'<sub>\e nm</sub>.
         * @param[in] N1 the maximum degree and order of the first correction
         *   coefficients \e C'<sub>\e nm</sub> and \e S'<sub>\e nm</sub>.
         * @param[in] C2 the coefficients \e C''<sub>\e nm</sub>.
         * @param[in] S2 the coefficients \e S''<sub>\e nm</sub>.
         * @param[in] N2 the maximum degree and order of the second correction
         *   coefficients \e C'<sub>\e nm</sub> and \e S'<sub>\e nm</sub>.
         * @param[in] a the reference radius appearing in the definition of the
         *   sum.
         * @param[in] norm the normalization for the associated Legendre
         *   polynomials, either SphericalHarmonic2::FULL (the default) or
         *   SphericalHarmonic2::SCHMIDT.
         * @exception GeographicErr if \e N and \e N1 do not satisfy \e N &ge;
         *   \e N1 &ge; &minus;1, and similarly for \e N2.
         * @exception GeographicErr if any of the vectors of coefficients is not
         *   large enough.
         *
         * See SphericalHarmonic for the way the coefficients should be stored.  \e
         * N1 and \e N2 should satisfy \e N1 &le; \e N and \e N2 &le; \e N.
         *
         * The class stores <i>pointers</i> to the first elements of \e C, \e S, \e
         * C', \e S', \e C'', and \e S''.  These arrays should not be altered or
         * destroyed during the lifetime of a SphericalHarmonic object.
         **********************************************************************/
        SphericalHarmonic2(array<double>^ C,
                           array<double>^ S,
                           int N,
                           array<double>^ C1,
                           array<double>^ S1,
                           int N1,
                           array<double>^ C2,
                           array<double>^ S2,
                           int N2,
                           double a,
                           Normalization norm );

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
         * @param[in] C1 the coefficients \e C'<sub>\e nm</sub>.
         * @param[in] S1 the coefficients \e S'<sub>\e nm</sub>.
         * @param[in] N1 the degree used to determine the layout of \e C' and \e
         *   S'.
         * @param[in] nmx1 the maximum degree used for \e C' and \e S'.
         * @param[in] mmx1 the maximum order used for \e C' and \e S'.
         * @param[in] C2 the coefficients \e C''<sub>\e nm</sub>.
         * @param[in] S2 the coefficients \e S''<sub>\e nm</sub>.
         * @param[in] N2 the degree used to determine the layout of \e C'' and \e
         *   S''.
         * @param[in] nmx2 the maximum degree used for \e C'' and \e S''.
         * @param[in] mmx2 the maximum order used for \e C'' and \e S''.
         * @param[in] a the reference radius appearing in the definition of the
         *   sum.
         * @param[in] norm the normalization for the associated Legendre
         *   polynomials, either SphericalHarmonic2::FULL (the default) or
         *   SphericalHarmonic2::SCHMIDT.
         * @exception GeographicErr if the parameters do not satisfy \e N &ge; \e
         *   nmx &ge; \e mmx &ge; &minus;1; \e N1 &ge; \e nmx1 &ge; \e mmx1 &ge;
         *   &minus;1; \e N &ge; \e N1; \e nmx &ge; \e nmx1; \e mmx &ge; \e mmx1;
         *   and similarly for \e N2, \e nmx2, and \e mmx2.
         * @exception GeographicErr if any of the vectors of coefficients is not
         *   large enough.
         *
         * The class stores <i>pointers</i> to the first elements of \e C, \e S, \e
         * C', \e S', \e C'', and \e S''.  These arrays should not be altered or
         * destroyed during the lifetime of a SphericalHarmonic object.
         **********************************************************************/
        SphericalHarmonic2(array<double>^ C,
                           array<double>^ S,
                           int N, int nmx, int mmx,
                           array<double>^ C1,
                           array<double>^ S1,
                           int N1, int nmx1, int mmx1,
                           array<double>^ C2,
                           array<double>^ S2,
                           int N2, int nmx2, int mmx2,
                           double a,
                           Normalization norm );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~SphericalHarmonic2()
        { this->!SphericalHarmonic2(); }

        /**
         * Compute a spherical harmonic sum with two correction terms.
         *
         * @param[in] tau1 multiplier for correction coefficients \e C' and \e S'.
         * @param[in] tau2 multiplier for correction coefficients \e C'' and \e S''.
         * @param[in] x cartesian coordinate.
         * @param[in] y cartesian coordinate.
         * @param[in] z cartesian coordinate.
         * @return \e V the spherical harmonic sum.
         *
         * This routine requires constant memory and thus never throws an
         * exception.
         **********************************************************************/
        double HarmonicSum(double tau1, double tau2, double x, double y, double z);

        /**
         * Compute a spherical harmonic sum with two correction terms and its
         * gradient.
         *
         * @param[in] tau1 multiplier for correction coefficients \e C' and \e S'.
         * @param[in] tau2 multiplier for correction coefficients \e C'' and \e S''.
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
        double HarmonicSum(double tau1, double tau2, double x, double y, double z,
                        [System::Runtime::InteropServices::Out] double% gradx,
                        [System::Runtime::InteropServices::Out] double% grady,
                        [System::Runtime::InteropServices::Out] double% gradz);

        /**
         * Create a CircularEngine to allow the efficient evaluation of several
         * points on a circle of latitude at fixed values of \e tau1 and \e tau2.
         *
         * @param[in] tau1 multiplier for correction coefficients \e C' and \e S'.
         * @param[in] tau2 multiplier for correction coefficients \e C'' and \e S''.
         * @param[in] p the radius of the circle.
         * @param[in] z the height of the circle above the equatorial plane.
         * @param[in] gradp if true the returned object will be able to compute the
         *   gradient of the sum.
         * @exception std::bad_alloc if the memory for the CircularEngine can't be
         *   allocated.
         * @return the CircularEngine object.
         *
         * SphericalHarmonic2::operator()() exchanges the order of the sums in the
         * definition, i.e., &sum;<sub>n = 0..N</sub> &sum;<sub>m = 0..n</sub>
         * becomes &sum;<sub>m = 0..N</sub> &sum;<sub>n = m..N</sub>..
         * SphericalHarmonic2::Circle performs the inner sum over degree \e n
         * (which entails about <i>N</i><sup>2</sup> operations).  Calling
         * CircularEngine::operator()() on the returned object performs the outer
         * sum over the order \e m (about \e N operations).
         *
         * See SphericalHarmonic::Circle for an example of its use.
         **********************************************************************/
        CircularEngine^ Circle(double tau1, double tau2, double p, double z, bool gradp);

        /**
        * @return the zeroth SphericalCoefficients object.
        **********************************************************************/
        SphericalCoefficients^ Coefficients();
        /**
        * @return the first SphericalCoefficients object.
        **********************************************************************/
        SphericalCoefficients^ Coefficients1();
        /**
        * @return the second SphericalCoefficients object.
        **********************************************************************/
        SphericalCoefficients^ Coefficients2();
    };
} // namespace NETGeographicLib
