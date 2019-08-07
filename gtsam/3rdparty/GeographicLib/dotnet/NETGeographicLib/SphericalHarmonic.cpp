/**
 * \file NETGeographicLib/SphericalHarmonic.cpp
 * \brief Implementation for NETGeographicLib::SphericalHarmonic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/SphericalHarmonic.hpp"
#include "SphericalHarmonic.h"
#include "CircularEngine.h"
#include "SphericalCoefficients.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::SphericalHarmonic";

//*****************************************************************************
SphericalHarmonic::!SphericalHarmonic(void)
{
    if ( m_pSphericalHarmonic != NULL )
    {
        delete m_pSphericalHarmonic;
        m_pSphericalHarmonic = NULL;
    }
    if ( m_C != NULL )
    {
        delete m_C;
        m_C = NULL;
    }
    if ( m_S != NULL )
    {
        delete m_S;
        m_S = NULL;
    }
}

//*****************************************************************************
SphericalHarmonic::SphericalHarmonic(array<double>^ C,
                    array<double>^ S,
                    int N, double a, Normalization norm )
{
    try
    {
        m_C = new std::vector<double>();
        m_S = new std::vector<double>();
        for each ( double x in C ) m_C->push_back(x);
        for each ( double x in S ) m_S->push_back(x);
        m_pSphericalHarmonic = new GeographicLib::SphericalHarmonic(
            *m_C, *m_S, N, a, static_cast<unsigned>(norm) );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
    catch ( System::Exception^ sxpt )
    {
        throw gcnew GeographicErr( sxpt->Message );
    }
}

//*****************************************************************************
SphericalHarmonic::SphericalHarmonic(array<double>^ C,
                    array<double>^ S,
                    int N, int nmx, int mmx,
                    double a, Normalization norm)
{
    try
    {
        m_C = new std::vector<double>();
        m_S = new std::vector<double>();
        for each ( double x in C ) m_C->push_back(x);
        for each ( double x in S ) m_S->push_back(x);
        m_pSphericalHarmonic = new GeographicLib::SphericalHarmonic(
            *m_C, *m_S, N, nmx, mmx, a, static_cast<unsigned>(norm) );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
    catch ( System::Exception^ sxpt )
    {
        throw gcnew GeographicErr( sxpt->Message );
    }
}

//*****************************************************************************
double SphericalHarmonic::HarmonicSum(double x, double y, double z)
{
    return m_pSphericalHarmonic->operator()( x, y, z );
}

//*****************************************************************************
double SphericalHarmonic::HarmonicSum(double x, double y, double z,
                        double% gradx, double% grady, double% gradz)
{
    double lx, ly, lz;
    double out = m_pSphericalHarmonic->operator()( x, y, z, lx, ly, lz );
    gradx = lx;
    grady = ly;
    gradz = lz;
    return out;
}

//*****************************************************************************
CircularEngine^ SphericalHarmonic::Circle(double p, double z, bool gradp)
{
    return gcnew CircularEngine( m_pSphericalHarmonic->Circle( p, z, gradp ) );
}

//*****************************************************************************
SphericalCoefficients^ SphericalHarmonic::Coefficients()
{
    return gcnew SphericalCoefficients( m_pSphericalHarmonic->Coefficients() );
}
