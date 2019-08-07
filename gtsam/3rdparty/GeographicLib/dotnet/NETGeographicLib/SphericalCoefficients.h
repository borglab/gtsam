#pragma once
/**
 * \file NETGeographicLib/SphericalCoefficients.h
 * \brief Header for NETGeographicLib::SphericalCoefficients class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    /*!
    \brief .NET wrapper for GeographicLib::SphericalEngine::coeff.

    This class allows .NET applications to access GeographicLib::SphericalEngine::coeff.

    The SphericalHarmonic classes provide accessor functions that allow
    you to examine the coefficients.  These accessor functions export a
    GeographicLib::SphericalEngine::coeff object.  The GeographicLib::SphericalEngine
    class is not implemented in NETGeographicLib.  SphericalCoefficients is provided as
    a substitute for GeographicLib::SphericalEngine::coeff allowing you to examine the
    coefficients in .NET applications.

    Use SphericalHarmonic::Coefficients, SphericalHarmonic1::Coefficient*,
    or SphericalHarmonic2::Coefficient* to obtain an instance of this
    class.

    <B>INTERFACE DIFFERENCES:</B><BR>
    This class does not implement readcoeffs.
    */
    public ref class SphericalCoefficients
    {
        private:
        // The cosine coefficients.
        array<double>^ m_C; // size = Csize(m_nmx,m_mmx)
        // The sine coefficients
        array<double>^ m_S; // size = Ssize(m_nmx,m_mmx)
        // The dimension of the coefficients
        int m_N;
        int m_nmx;
        int m_mmx;
    public:
        /*!
        \brief Constructor.
        \param[in] c A reference to a GeographicLib::SphericalEngine::coeff object.
        This constructor is for internal use only.  Developers should
        not create an instance of SphericalCoefficients.  Use
        SphericalHarmonic::Coefficients, SphericalHarmonic1::Coefficient*,
        or SphericalHarmonic2::Coefficient* to obtain an instance of this
        class.
        */
        SphericalCoefficients( const GeographicLib::SphericalEngine::coeff& c );

        /**
        * @return \e N the degree giving storage layout for \e C and \e S.
        **********************************************************************/
        property int N { int get() { return m_N; } }
        /**
        * @return \e nmx the maximum degree to be used.
        **********************************************************************/
        property int nmx { int get() { return m_nmx; } }
        /**
        * @return \e mmx the maximum order to be used.
        **********************************************************************/
        property int mmx { int get() { return m_mmx; } }
        /**
        * The one-dimensional index into \e C and \e S.
        *
        * @param[in] n the degree.
        * @param[in] m the order.
        * @return the one-dimensional index.
        **********************************************************************/
        int index(int n, int m)
        { return m * m_N - m * (m - 1) / 2 + n; }
        /**
        * An element of \e C.
        *
        * @param[in] k the one-dimensional index.
        * @return the value of the \e C coefficient.
        **********************************************************************/
        double Cv(int k) { return m_C[k]; }
        /**
        * An element of \e S.
        *
        * @param[in] k the one-dimensional index.
        * @return the value of the \e S coefficient.
        **********************************************************************/
        double Sv(int k) { return m_S[k - (m_N + 1)]; }
        /**
        * An element of \e C with checking.
        *
        * @param[in] k the one-dimensional index.
        * @param[in] n the requested degree.
        * @param[in] m the requested order.
        * @param[in] f a multiplier.
        * @return the value of the \e C coefficient multiplied by \e f in \e n
        *   and \e m are in range else 0.
        **********************************************************************/
        double Cv(int k, int n, int m, double f)
        { return m > m_mmx || n > m_nmx ? 0 : m_C[k] * f; }
        /**
        * An element of \e S with checking.
        *
        * @param[in] k the one-dimensional index.
        * @param[in] n the requested degree.
        * @param[in] m the requested order.
        * @param[in] f a multiplier.
        * @return the value of the \e S coefficient multiplied by \e f in \e n
        *   and \e m are in range else 0.
        **********************************************************************/
        double Sv(int k, int n, int m, double f)
        { return m > m_mmx || n > m_nmx ? 0 : m_S[k - (m_N + 1)] * f; }

        /**
        * The size of the coefficient vector for the cosine terms.
        *
        * @param[in] N the maximum degree.
        * @param[in] M the maximum order.
        * @return the size of the vector of cosine terms as stored in column
        *   major order.
        **********************************************************************/
        static int Csize(int N, int M)
        { return (M + 1) * (2 * N - M + 2) / 2; }

        /**
        * The size of the coefficient vector for the sine terms.
        *
        * @param[in] N the maximum degree.
        * @param[in] M the maximum order.
        * @return the size of the vector of cosine terms as stored in column
        *   major order.
        **********************************************************************/
        static int Ssize(int N, int M)
        { return Csize(N, M) - (N + 1); }

    };
} // namespace NETGeographicLib
