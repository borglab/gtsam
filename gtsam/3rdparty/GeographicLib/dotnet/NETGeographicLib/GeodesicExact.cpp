/**
 * \file NETGeographicLib/GeodesicExact.cpp
 * \brief Implementation for NETGeographicLib::GeodesicExact class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GeodesicExact.hpp"
#include "GeographicLib/GeodesicLineExact.hpp"
#include "GeodesicExact.h"
#include "GeodesicLineExact.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::GeodesicExact";

//*****************************************************************************
GeodesicExact::!GeodesicExact(void)
{
    if ( m_pGeodesicExact != NULL )
    {
        delete m_pGeodesicExact;
        m_pGeodesicExact = NULL;
    }
}

//*****************************************************************************
GeodesicExact::GeodesicExact()
{
    try
    {
        m_pGeodesicExact = new GeographicLib::GeodesicExact( GeographicLib::GeodesicExact::WGS84() );
    }
    catch ( std::bad_alloc err )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
GeodesicExact::GeodesicExact(double a, double f)
{
    try
    {
        m_pGeodesicExact = new GeographicLib::GeodesicExact( a, f );
    }
    catch ( std::bad_alloc err )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2, lazi2, lm12, lM12, lM21, lS12 );
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
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
    return out;
}

//*****************************************************************************
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, lm12;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2, lazi2, lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lM12, lM21;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2, lazi2, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double GeodesicExact::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesicExact->Direct( lat1, lon1, azi1, s12,
                llat2, llon2, lazi2, lm12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
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
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12 );
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
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12)
{
    double llat2, llon2, lazi2, ls12;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2, ls12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, ls12, lm12;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2, ls12, lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% M12,
                [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lM12, lM21;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2, ls12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
void GeodesicExact::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% m12,
                [System::Runtime::InteropServices::Out] double% M12,
                [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21;
    m_pGeodesicExact->ArcDirect( lat1, lon1, azi1, a12,
                llat2, llon2, lazi2, ls12, lm12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
double GeodesicExact::GenDirect(double lat1, double lon1, double azi1,
                        bool arcmode, double s12_a12,
                        GeodesicExact::mask outmask,
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
    double out = m_pGeodesicExact->GenDirect( lat1, lon1, azi1, arcmode, s12_a12,
                static_cast<unsigned>(outmask), llat2, llon2, lazi2, ls12, lm12, lM12,
                lM21, lS12 );
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
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2, lm12, lM12, lM21, lS12 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12)
{
    double ls12;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2, ls12 );
    s12 = ls12;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double lazi1, lazi2;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                lazi1, lazi2 );
    azi1 = lazi1;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double ls12, lazi1, lazi2;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12)
{
    double ls12, lazi1, lazi2, lm12;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2, lm12 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double ls12, lazi1, lazi2, lM12, lM21;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2, lM12, lM21 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double GeodesicExact::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesicExact->Inverse( lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2, lm12, lM12, lM21 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double GeodesicExact::GenInverse(double lat1, double lon1, double lat2, double lon2,
                        GeodesicExact::mask outmask,
                        [System::Runtime::InteropServices::Out] double% s12,
                        [System::Runtime::InteropServices::Out] double% azi1,
                        [System::Runtime::InteropServices::Out] double% azi2,
                        [System::Runtime::InteropServices::Out] double% m12,
                        [System::Runtime::InteropServices::Out] double% M12,
                        [System::Runtime::InteropServices::Out] double% M21,
                        [System::Runtime::InteropServices::Out] double% S12)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesicExact->GenInverse( lat1, lon1, lat2, lon2,
                static_cast<unsigned>(outmask), ls12, lazi1, lazi2, lm12, lM12,
                lM21, lS12 );
    s12 = ls12;
    azi1 = lazi1;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
    return out;
}

//*****************************************************************************
System::IntPtr^ GeodesicExact::GetUnmanaged()
{
    return gcnew System::IntPtr( const_cast<void*>(
        reinterpret_cast<const void*>(m_pGeodesicExact) ) );
}

//*****************************************************************************
GeodesicLineExact^ GeodesicExact::Line(double lat1, double lon1, double azi1,
    NETGeographicLib::Mask caps )
{
    return gcnew GeodesicLineExact( this, lat1, lon1, azi1, caps );
}

//*****************************************************************************
GeodesicLineExact^ GeodesicExact::InverseLine(double lat1, double lon1,
    double lat2, double lon2, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLineExact(m_pGeodesicExact->InverseLine(
        lat1, lon1, lat2, lon2, static_cast<unsigned>(caps)));
}

//*****************************************************************************
GeodesicLineExact^ GeodesicExact::DirectLine(double lat1, double lon1,
    double azi1, double s12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLineExact(m_pGeodesicExact->DirectLine(
        lat1, lon1, azi1, s12, static_cast<unsigned>(caps)));
}

//*****************************************************************************
GeodesicLineExact^ GeodesicExact::ArcDirectLine(double lat1, double lon1,
    double azi1, double a12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLineExact(m_pGeodesicExact->ArcDirectLine(
        lat1, lon1, azi1, a12, static_cast<unsigned>(caps)));
}

//*****************************************************************************
GeodesicLineExact^ GeodesicExact::GenDirectLine(double lat1, double lon1,
    double azi1, bool arcmode, double s12_a12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLineExact(m_pGeodesicExact->GenDirectLine(
        lat1, lon1, azi1, arcmode, s12_a12, static_cast<unsigned>(caps)));
}

//*****************************************************************************
double GeodesicExact::MajorRadius::get()
{ return m_pGeodesicExact->MajorRadius(); }

//*****************************************************************************
double GeodesicExact::Flattening::get()
{ return m_pGeodesicExact->Flattening(); }

//*****************************************************************************
double GeodesicExact::EllipsoidArea::get()
{ return m_pGeodesicExact->EllipsoidArea(); }
