/**
 * \file NETGeographicLib/SphericalHarmonic2.cpp
 * \brief Implementation for NETGeographicLib::SphericalHarmonic2 class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/SphericalHarmonic2.hpp"
#include "SphericalHarmonic2.h"
#include "CircularEngine.h"
#include "SphericalCoefficients.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::SphericalHarmonic2";

//*****************************************************************************
SphericalHarmonic2::!SphericalHarmonic2(void)
{
    if ( m_pSphericalHarmonic2 != NULL )
    {
        delete m_pSphericalHarmonic2;
        m_pSphericalHarmonic2 = NULL;
    }

    if ( m_C != NULL )
    {
        for ( int i = 0; i < m_numCoeffVectors; i++ )
            if ( m_C[i] != NULL )
                delete m_C[i];
        delete [] m_C;
        m_C = NULL;
    }

    if ( m_S != NULL )
    {
        for ( int i = 0; i < m_numCoeffVectors; i++ )
            if ( m_S[i] != NULL )
                delete m_S[i];
        delete [] m_S;
        m_S = NULL;
    }
}

//*****************************************************************************
SphericalHarmonic2::SphericalHarmonic2(array<double>^ C,
                                        array<double>^ S,
                                        int N,
                                        array<double>^ C1,
                                        array<double>^ S1,
                                        int N1,
                                        array<double>^ C2,
                                        array<double>^ S2,
                                        int N2,
                                        double a,
                                        Normalization norm )
{
    try
    {
        m_C = new std::vector<double>*[m_numCoeffVectors];
        for ( int i = 0; i < m_numCoeffVectors; i++ ) m_C[i] = new std::vector<double>();
        m_S = new std::vector<double>*[m_numCoeffVectors];
        for ( int i = 0; i < m_numCoeffVectors; i++ ) m_S[i] = new std::vector<double>();
        for each ( double x in C ) m_C[0]->push_back( x );
        for each ( double x in S ) m_S[0]->push_back( x );
        for each ( double x in C1 ) m_C[1]->push_back( x );
        for each ( double x in S1 ) m_S[1]->push_back( x );
        for each ( double x in C2 ) m_C[2]->push_back( x );
        for each ( double x in S2 ) m_S[2]->push_back( x );
        m_pSphericalHarmonic2 = new GeographicLib::SphericalHarmonic2( *m_C[0], *m_S[0], N,
            *m_C[1], *m_S[1], N1, *m_C[2], *m_S[2], N2, a, (unsigned)norm);
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( System::Exception^ sxpt )
    {
        throw gcnew GeographicErr( sxpt->Message );
    }
}

//*****************************************************************************
SphericalHarmonic2::SphericalHarmonic2(array<double>^ C,
                                        array<double>^ S,
                                        int N, int nmx, int mmx,
                                        array<double>^ C1,
                                        array<double>^ S1,
                                        int N1, int nmx1, int mmx1,
                                        array<double>^ C2,
                                        array<double>^ S2,
                                        int N2, int nmx2, int mmx2,
                                        double a,
                                        Normalization norm )
{
    try
    {
        m_C = new std::vector<double>*[m_numCoeffVectors];
        for ( int i = 0; i < m_numCoeffVectors; i++ ) m_C[i] = new std::vector<double>();
        m_S = new std::vector<double>*[m_numCoeffVectors];
        for ( int i = 0; i < m_numCoeffVectors; i++ ) m_S[i] = new std::vector<double>();
        for each ( double x in C ) m_C[0]->push_back( x );
        for each ( double x in S ) m_S[0]->push_back( x );
        for each ( double x in C1 ) m_C[1]->push_back( x );
        for each ( double x in S1 ) m_S[1]->push_back( x );
        for each ( double x in C2 ) m_C[2]->push_back( x );
        for each ( double x in S2 ) m_S[2]->push_back( x );
        m_pSphericalHarmonic2 = new GeographicLib::SphericalHarmonic2( *m_C[0], *m_S[0], N,
            nmx, mmx, *m_C[1], *m_S[1], N1, nmx1, mmx1, *m_C[2], *m_S[2], N2, nmx2, mmx2, a,
            (unsigned)norm );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( System::Exception^ sxpt )
    {
        throw gcnew GeographicErr( sxpt->Message );
    }
}

//*****************************************************************************
double SphericalHarmonic2::HarmonicSum(double tau1, double tau2, double x, double y, double z)
{
    return m_pSphericalHarmonic2->operator()( tau1, tau2, x, y, z );
}

//*****************************************************************************
double SphericalHarmonic2::HarmonicSum(double tau1, double tau2, double x, double y, double z,
                        [System::Runtime::InteropServices::Out] double% gradx,
                        [System::Runtime::InteropServices::Out] double% grady,
                        [System::Runtime::InteropServices::Out] double% gradz)
{
    double lgradx, lgrady, lgradz;
    double out = m_pSphericalHarmonic2->operator()( tau1, tau2, x, y, z,
        lgradx, lgrady, lgradz );
    gradx = lgradx;
    grady = lgrady;
    gradz = lgradz;
    return out;
}

//*****************************************************************************
CircularEngine^ SphericalHarmonic2::Circle(double tau1, double tau2, double p,
    double z, bool gradp)
{
    try
    {
        return gcnew CircularEngine( m_pSphericalHarmonic2->Circle( tau1, tau2, p, z, gradp ) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Memory allocation error in SphericalHarmonic2::Circle" );
    }
}

//*****************************************************************************
SphericalCoefficients^ SphericalHarmonic2::Coefficients()
{
    return gcnew SphericalCoefficients( m_pSphericalHarmonic2->Coefficients() );
}

//*****************************************************************************
SphericalCoefficients^ SphericalHarmonic2::Coefficients1()
{
    return gcnew SphericalCoefficients( m_pSphericalHarmonic2->Coefficients1() );
}

//*****************************************************************************
SphericalCoefficients^ SphericalHarmonic2::Coefficients2()
{
    return gcnew SphericalCoefficients( m_pSphericalHarmonic2->Coefficients2() );
}
