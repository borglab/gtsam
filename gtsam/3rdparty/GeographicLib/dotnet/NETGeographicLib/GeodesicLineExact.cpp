/**
 * \file NETGeographicLib/GeodesicLineExact.cpp
 * \brief Implementation for NETGeographicLib::GeodesicLineExact class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GeodesicLineExact.hpp"
#include "GeodesicLineExact.h"
#include "GeodesicExact.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::GeodesicLineExact";

//*****************************************************************************
GeodesicLineExact::!GeodesicLineExact(void)
{
    if ( m_pGeodesicLineExact != NULL )
    {
        delete m_pGeodesicLineExact;
        m_pGeodesicLineExact = NULL;
    }
}

//*****************************************************************************
GeodesicLineExact::GeodesicLineExact(GeodesicExact^ g, double lat1,
    double lon1, double azi1, NETGeographicLib::Mask caps )
{
    try
    {
        const GeographicLib::GeodesicExact* pGeodesicExact=
            reinterpret_cast<const GeographicLib::GeodesicExact*>(
                g->GetUnmanaged()->ToPointer() );
        m_pGeodesicLineExact = new GeographicLib::GeodesicLineExact(
            *pGeodesicExact, lat1, lon1, azi1, static_cast<unsigned>(caps) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
GeodesicLineExact::GeodesicLineExact(
    const GeographicLib::GeodesicLineExact& gle)
{
    try
    {
        m_pGeodesicLineExact = new GeographicLib::GeodesicLineExact(gle);
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr(BADALLOC);
    }
}

//*****************************************************************************
GeodesicLineExact::GeodesicLineExact(double lat1, double lon1, double azi1,
                                     NETGeographicLib::Mask caps)
{
    try
    {
        m_pGeodesicLineExact = new GeographicLib::GeodesicLineExact(
            GeographicLib::GeodesicExact::WGS84(), lat1, lon1, azi1,
            static_cast<unsigned>(caps) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
        [System::Runtime::InteropServices::Out] double% lat2,
        [System::Runtime::InteropServices::Out] double% lon2,
        [System::Runtime::InteropServices::Out] double% azi2,
        [System::Runtime::InteropServices::Out] double% m12,
        [System::Runtime::InteropServices::Out] double% M12,
        [System::Runtime::InteropServices::Out] double% M21,
        [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2, lazi2,
        lm12, lM12, lM21, lS12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2);
    lat2 = llat2;
    lon2 = llon2;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2, lazi2);
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, lm12;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2, lazi2,
        lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lM12, lM21;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2, lazi2,
        lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesicLineExact->Position( s12, llat2, llon2, lazi2,
        lm12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21,
    [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2, ls12,
        lm12, lM12, lM21, lS12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12)
{
    double llat2, llon2, lazi2, ls12;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2, ls12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, ls12, lm12;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2, ls12,
        lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lM12, lM21;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2, ls12,
        lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
void GeodesicLineExact::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21;
    m_pGeodesicLineExact->ArcPosition( a12, llat2, llon2, lazi2, ls12,
        lm12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
double GeodesicLineExact::GenPosition(bool arcmode, double s12_a12,
    GeodesicLineExact::mask outmask,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21,
    [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicLineExact->GenPosition( arcmode, s12_a12,
        static_cast<unsigned>(outmask),
        llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
    return out;
}

//*****************************************************************************
double GeodesicLineExact::Latitude::get()
{ return m_pGeodesicLineExact->Latitude(); }

//*****************************************************************************
double GeodesicLineExact::Longitude::get()
{ return m_pGeodesicLineExact->Longitude(); }

//*****************************************************************************
double GeodesicLineExact::Azimuth::get()
{ return m_pGeodesicLineExact->Azimuth(); }

//*****************************************************************************
double GeodesicLineExact::EquatorialAzimuth::get()
{ return m_pGeodesicLineExact->EquatorialAzimuth(); }

//*****************************************************************************
double GeodesicLineExact::EquatorialArc::get()
{ return m_pGeodesicLineExact->EquatorialArc(); }

//*****************************************************************************
double GeodesicLineExact::MajorRadius::get()
{ return m_pGeodesicLineExact->MajorRadius(); }

//*****************************************************************************
double GeodesicLineExact::Flattening::get()
{ return m_pGeodesicLineExact->Flattening(); }

//*****************************************************************************
double GeodesicLineExact::Distance::get()
{ return m_pGeodesicLineExact->Distance(); }

//*****************************************************************************
double GeodesicLineExact::Arc::get()
{ return m_pGeodesicLineExact->Arc(); }

//*****************************************************************************
NETGeographicLib::Mask GeodesicLineExact::Capabilities()
{ return static_cast<NETGeographicLib::Mask>(m_pGeodesicLineExact->Capabilities()); }

//*****************************************************************************
bool GeodesicLineExact::Capabilities(NETGeographicLib::Mask testcaps)
{ return m_pGeodesicLineExact->Capabilities(static_cast<unsigned>(testcaps)); }

//*****************************************************************************
void GeodesicLineExact::SetDistance(double s13)
{ m_pGeodesicLineExact->SetDistance(s13); }

//*****************************************************************************
void GeodesicLineExact::SetArc(double a13)
{ m_pGeodesicLineExact->SetArc(a13); }

//*****************************************************************************
void GeodesicLineExact::GenSetDistance(bool arcmode, double s13_a13)
{ m_pGeodesicLineExact->GenSetDistance(arcmode, s13_a13); }

//*****************************************************************************
void GeodesicLineExact::AzimuthSinCos(double% sazi1, double% cazi1)
{
    double x1, x2;
    m_pGeodesicLineExact->Azimuth(x1, x2);
    sazi1 = x1;
    cazi1 = x2;
}

//*****************************************************************************
void GeodesicLineExact::EquatorialAzimuthSinCos(double% sazi0, double% cazi0)
{
    double x1, x2;
    m_pGeodesicLineExact->EquatorialAzimuth(x1, x2);
    sazi0 = x1;
    cazi0 = x2;
}

//*****************************************************************************
double GeodesicLineExact::GenDistance(bool arcmode)
{ return m_pGeodesicLineExact->GenDistance(arcmode); }
