/**
 * \file NETGeographicLib/GravityCircle.cpp
 * \brief Implementation for NETGeographicLib::GravityCircle class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GravityCircle.hpp"
#include "GravityModel.h"
#include "GravityCircle.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
GravityCircle::!GravityCircle(void)
{
    if ( m_pGravityCircle != NULL )
    {
        delete m_pGravityCircle;
        m_pGravityCircle = NULL;
    }
}

//*****************************************************************************
GravityCircle::GravityCircle( const GeographicLib::GravityCircle& gc )
{
    try
    {
        m_pGravityCircle = new GeographicLib::GravityCircle(gc);
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::GravityCircle" );
    }
}

//*****************************************************************************
double GravityCircle::Gravity(double lon,
    [System::Runtime::InteropServices::Out] double% gx,
    [System::Runtime::InteropServices::Out] double% gy,
    [System::Runtime::InteropServices::Out] double% gz)
{
    double lgx, lgy, lgz;
    double out = m_pGravityCircle->Gravity( lon, lgx, lgy, lgz );
    gx = lgx;
    gy = lgy;
    gz = lgz;
    return out;
}

//*****************************************************************************
double GravityCircle::Disturbance(double lon,
    [System::Runtime::InteropServices::Out] double% deltax,
    [System::Runtime::InteropServices::Out] double% deltay,
    [System::Runtime::InteropServices::Out] double% deltaz)
{
    double ldeltax, ldeltay, ldeltaz;
    double out = m_pGravityCircle->Disturbance( lon, ldeltax, ldeltay, ldeltaz );
    deltax = ldeltax;
    deltay = ldeltay;
    deltaz = ldeltaz;
    return out;
}

//*****************************************************************************
double GravityCircle::GeoidHeight(double lon)
{
    return m_pGravityCircle->GeoidHeight( lon );
}

//*****************************************************************************
void GravityCircle::SphericalAnomaly(double lon,
    [System::Runtime::InteropServices::Out] double% Dg01,
    [System::Runtime::InteropServices::Out] double% xi,
    [System::Runtime::InteropServices::Out] double% eta)
{
    double lDg01, lxi, leta;
    m_pGravityCircle->SphericalAnomaly( lon, lDg01, lxi, leta );
    Dg01 = lDg01;
    xi = lxi;
    eta = leta;
}

//*****************************************************************************
double GravityCircle::W(double lon,
    [System::Runtime::InteropServices::Out] double% gX,
    [System::Runtime::InteropServices::Out] double% gY,
    [System::Runtime::InteropServices::Out] double% gZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityCircle->W( lon, lgx, lgy, lgz );
    gX = lgx;
    gY = lgy;
    gZ = lgz;
    return out;
}

//*****************************************************************************
double GravityCircle::V(double lon,
    [System::Runtime::InteropServices::Out] double% GX,
    [System::Runtime::InteropServices::Out] double% GY,
    [System::Runtime::InteropServices::Out] double% GZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityCircle->V( lon, lgx, lgy, lgz );
    GX = lgx;
    GY = lgy;
    GZ = lgz;
    return out;
}

//*****************************************************************************
double GravityCircle::T(double lon,
    [System::Runtime::InteropServices::Out] double% deltaX,
    [System::Runtime::InteropServices::Out] double% deltaY,
    [System::Runtime::InteropServices::Out] double% deltaZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityCircle->T( lon, lgx, lgy, lgz );
    deltaX = lgx;
    deltaY = lgy;
    deltaZ = lgz;
    return out;
}

//*****************************************************************************
double GravityCircle::T(double lon)
{
    return m_pGravityCircle->T( lon );
}

//*****************************************************************************
double GravityCircle::MajorRadius::get()
{
    if ( m_pGravityCircle->Init() )
        return m_pGravityCircle->MajorRadius();
    throw gcnew GeographicErr("GravityCircle::MajorRadius failed because the GravityCircle is not initialized.");
}

//*****************************************************************************
double GravityCircle::Flattening::get()
{
    if ( m_pGravityCircle->Init() )
        return m_pGravityCircle->Flattening();
    throw gcnew GeographicErr("GravityCircle::Flattening failed because the GravityCircle is not initialized.");
}

//*****************************************************************************
double GravityCircle::Latitude::get()
{
    if ( m_pGravityCircle->Init() )
        return m_pGravityCircle->Latitude();
    throw gcnew GeographicErr("GravityCircle::Latitude failed because the GravityCircle is not initialized.");
}

//*****************************************************************************
double GravityCircle::Height::get()
{
    if ( m_pGravityCircle->Init() )
        return m_pGravityCircle->Height();
    throw gcnew GeographicErr("GravityCircle::Height failed because the GravityCircle is not initialized.");
}

//*****************************************************************************
bool GravityCircle::Init::get() { return m_pGravityCircle->Init(); }

//*****************************************************************************
GravityModel::Mask GravityCircle::Capabilities()
{ return static_cast<GravityModel::Mask>(m_pGravityCircle->Capabilities()); }

//*****************************************************************************
bool GravityCircle::Capabilities(GravityModel::Mask testcaps)
{ return m_pGravityCircle->Capabilities( (unsigned int)testcaps ); }
