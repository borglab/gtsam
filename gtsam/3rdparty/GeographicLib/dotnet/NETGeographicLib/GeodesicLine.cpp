/**
 * \file NETGeographicLib/GeodesicLine.cpp
 * \brief Implementation for NETGeographicLib::GeodesicLine class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GeodesicLine.hpp"
#include "Geodesic.h"
#include "GeodesicLine.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::GeodesicLine";

//*****************************************************************************
GeodesicLine::!GeodesicLine(void)
{
    if ( m_pGeodesicLine != NULL )
    {
        delete m_pGeodesicLine;
        m_pGeodesicLine = NULL;
    }
}

//*****************************************************************************
GeodesicLine::GeodesicLine( Geodesic^ g, double lat1, double lon1, double azi1,
                NETGeographicLib::Mask caps )
{
    try
    {
        const GeographicLib::Geodesic* pGeodesic =
            reinterpret_cast<const GeographicLib::Geodesic*>(
                g->GetUnmanaged()->ToPointer() );
        m_pGeodesicLine = new GeographicLib::GeodesicLine( *pGeodesic,
            lat1, lon1, azi1, static_cast<unsigned>(caps) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
GeodesicLine::GeodesicLine(const GeographicLib::GeodesicLine& gl)
{
    try
    {
        m_pGeodesicLine = new GeographicLib::GeodesicLine(gl);
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr(BADALLOC);
    }
}

//*****************************************************************************
GeodesicLine::GeodesicLine(double lat1, double lon1, double azi1,
                           NETGeographicLib::Mask caps)
{
    try
    {
        m_pGeodesicLine = new GeographicLib::GeodesicLine(
            GeographicLib::Geodesic::WGS84(), lat1, lon1, azi1,
            static_cast<unsigned>(caps) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
double GeodesicLine::Position(double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2, lazi2, lm12,
        lM12, lM21, lS12 );
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
double GeodesicLine::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2);
    lat2 = llat2;
    lon2 = llon2;
    return out;
}

//*****************************************************************************
double GeodesicLine::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double GeodesicLine::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, lm12;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2, lazi2,
                                            lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double GeodesicLine::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lM12, lM21;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2, lazi2,
        lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double GeodesicLine::Position(double s12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesicLine->Position( s12, llat2, llon2, lazi2, lm12,
        lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
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
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2,
        ls12, lm12, lM12, lM21, lS12 );
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
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12)
{
    double llat2, llon2, lazi2, ls12;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2, ls12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, ls12, lm12;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2, ls12, lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lM12, lM21;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2,
        ls12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
void GeodesicLine::ArcPosition(double a12,
    [System::Runtime::InteropServices::Out] double% lat2,
    [System::Runtime::InteropServices::Out] double% lon2,
    [System::Runtime::InteropServices::Out] double% azi2,
    [System::Runtime::InteropServices::Out] double% s12,
    [System::Runtime::InteropServices::Out] double% m12,
    [System::Runtime::InteropServices::Out] double% M12,
    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21;
    m_pGeodesicLine->ArcPosition( a12, llat2, llon2, lazi2,
        ls12, lm12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
double GeodesicLine::GenPosition(bool arcmode, double s12_a12,
    GeodesicLine::mask outmask,
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
    double out = m_pGeodesicLine->GenPosition( arcmode, s12_a12,
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
double GeodesicLine::Latitude::get() { return m_pGeodesicLine->Latitude(); }

//*****************************************************************************
double GeodesicLine::Longitude::get() { return m_pGeodesicLine->Longitude(); }

//*****************************************************************************
double GeodesicLine::Azimuth::get() { return m_pGeodesicLine->Azimuth(); }

//*****************************************************************************
double GeodesicLine::EquatorialAzimuth::get()
{ return m_pGeodesicLine->EquatorialAzimuth(); }

//*****************************************************************************
double GeodesicLine::EquatorialArc::get()
{ return m_pGeodesicLine->EquatorialArc(); }

//*****************************************************************************
double GeodesicLine::MajorRadius::get()
{ return m_pGeodesicLine->MajorRadius(); }

//*****************************************************************************
double GeodesicLine::Flattening::get()
{ return m_pGeodesicLine->Flattening(); }

//*****************************************************************************
double GeodesicLine::Distance::get()
{ return m_pGeodesicLine->Distance(); }

//*****************************************************************************
double GeodesicLine::Arc::get()
{ return m_pGeodesicLine->Arc(); }

//*****************************************************************************
NETGeographicLib::Mask GeodesicLine::Capabilities()
{ return static_cast<NETGeographicLib::Mask>(m_pGeodesicLine->Capabilities()); }

//*****************************************************************************
bool GeodesicLine::Capabilities(GeodesicLine::mask testcaps)
{ return m_pGeodesicLine->Capabilities( static_cast<unsigned>(testcaps) ); }

//*****************************************************************************
void GeodesicLine::SetDistance(double s13)
{ m_pGeodesicLine->SetDistance(s13); }

//*****************************************************************************
void GeodesicLine::SetArc(double a13)
{ m_pGeodesicLine->SetArc(a13); }

//*****************************************************************************
void GeodesicLine::GenSetDistance(bool arcmode, double s13_a13)
{ m_pGeodesicLine->GenSetDistance(arcmode, s13_a13); }

//*****************************************************************************
void GeodesicLine::AzimuthSinCos(double% sazi1, double% cazi1)
{
    double x1, x2;
    m_pGeodesicLine->Azimuth(x1, x2);
    sazi1 = x1;
    cazi1 = x2;
}

//*****************************************************************************
void GeodesicLine::EquatorialAzimuthSinCos(double% sazi0, double% cazi0)
{
    double x1, x2;
    m_pGeodesicLine->EquatorialAzimuth(x1, x2);
    sazi0 = x1;
    cazi0 = x2;
}

//*****************************************************************************
double GeodesicLine::GenDistance(bool arcmode)
{ return m_pGeodesicLine->GenDistance(arcmode); }
