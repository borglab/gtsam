/**
 * \file NETGeographicLib/NormalGravity.cpp
 * \brief Implementation for NETGeographicLib::NormalGravity class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/NormalGravity.hpp"
#include "NormalGravity.h"
#include "Geocentric.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::NormalGravity";

//*****************************************************************************
NormalGravity::!NormalGravity(void)
{
    if ( m_pNormalGravity != NULL )
    {
        delete m_pNormalGravity;
        m_pNormalGravity = NULL;
    }
}

//*****************************************************************************
NormalGravity::NormalGravity(double a, double GM, double omega, double f_J2, bool geometricp)
{
    try
    {
      m_pNormalGravity = new GeographicLib::NormalGravity( a, GM, omega, f_J2, geometricp );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
NormalGravity::NormalGravity(StandardModels model)
{
    try
    {
        m_pNormalGravity = model == StandardModels::WGS84 ?
            new GeographicLib::NormalGravity( GeographicLib::NormalGravity::WGS84() ) :
            new GeographicLib::NormalGravity( GeographicLib::NormalGravity::GRS80() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
NormalGravity::NormalGravity( const GeographicLib::NormalGravity& g)
{
    try
    {
        m_pNormalGravity = new GeographicLib::NormalGravity( g );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
double NormalGravity::SurfaceGravity(double lat)
{
    return m_pNormalGravity->SurfaceGravity( lat );
}

//*****************************************************************************
double NormalGravity::Gravity(double lat, double h,
    [System::Runtime::InteropServices::Out] double% gammay,
    [System::Runtime::InteropServices::Out] double% gammaz)
{
    double ly, lz;
    double out = m_pNormalGravity->Gravity( lat, h, ly, lz );
    gammay = ly;
    gammaz = lz;
    return out;
}

//*****************************************************************************
double NormalGravity::U(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% gammaX,
                [System::Runtime::InteropServices::Out] double% gammaY,
                [System::Runtime::InteropServices::Out] double% gammaZ)
{
    double lx, ly, lz;
    double out = m_pNormalGravity->U( X, Y, Z, lx, ly, lz );
    gammaX = lx;
    gammaY = ly;
    gammaZ = lz;
    return out;
}

//*****************************************************************************
double NormalGravity::V0(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% GammaX,
                [System::Runtime::InteropServices::Out] double% GammaY,
                [System::Runtime::InteropServices::Out] double% GammaZ)
{
    double lx, ly, lz;
    double out = m_pNormalGravity->V0( X, Y, Z, lx, ly, lz );
    GammaX = lx;
    GammaY = ly;
    GammaZ = lz;
    return out;
}

//*****************************************************************************
double NormalGravity::Phi(double X, double Y,
    [System::Runtime::InteropServices::Out] double% fX,
    [System::Runtime::InteropServices::Out] double% fY)
{
    double lx, ly;
    double out = m_pNormalGravity->Phi( X, Y, lx, ly );
    fX = lx;
    fY = ly;
    return out;
}

//*****************************************************************************
Geocentric^ NormalGravity::Earth()
{
    return gcnew Geocentric( m_pNormalGravity->Earth() );
}

//*****************************************************************************
double NormalGravity::MajorRadius::get()
{ return m_pNormalGravity->MajorRadius(); }

//*****************************************************************************
double NormalGravity::MassConstant::get()
{ return m_pNormalGravity->MassConstant(); }

//*****************************************************************************
double NormalGravity::DynamicalFormFactor(int n)
{ return m_pNormalGravity->DynamicalFormFactor(n); }

//*****************************************************************************
double NormalGravity::AngularVelocity::get()
{ return m_pNormalGravity->AngularVelocity(); }

//*****************************************************************************
double NormalGravity::Flattening::get()
{ return m_pNormalGravity->Flattening(); }

//*****************************************************************************
double NormalGravity::EquatorialGravity::get()
{ return m_pNormalGravity->EquatorialGravity(); }

//*****************************************************************************
double NormalGravity::PolarGravity::get()
{ return m_pNormalGravity->PolarGravity(); }

//*****************************************************************************
double NormalGravity::GravityFlattening::get()
{ return m_pNormalGravity->GravityFlattening(); }

//*****************************************************************************
double NormalGravity::SurfacePotential::get()
{ return m_pNormalGravity->SurfacePotential(); }

//*****************************************************************************
NormalGravity^ NormalGravity::WGS84()
{
    return gcnew NormalGravity( StandardModels::WGS84 );
}

//*****************************************************************************
NormalGravity^ NormalGravity::GRS80()
{
    return gcnew NormalGravity( StandardModels::GRS80 );
}

//*****************************************************************************
double NormalGravity::J2ToFlattening(double a, double GM, double omega,
                                     double J2)
{
    return GeographicLib::NormalGravity::J2ToFlattening( a, GM, omega, J2);
}

//*****************************************************************************
double NormalGravity::FlatteningToJ2(double a, double GM, double omega,
                                     double f)
{
    return GeographicLib::NormalGravity::FlatteningToJ2( a, GM, omega, f);
}
