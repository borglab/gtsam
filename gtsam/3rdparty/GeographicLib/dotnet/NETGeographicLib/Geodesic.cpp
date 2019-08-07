/**
 * \file NETGeographicLib/Geodesic.cpp
 * \brief Implementation for NETGeographicLib::Geodesic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include "Geodesic.h"
#include "GeodesicLine.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::Geodesic";

//*****************************************************************************
Geodesic::!Geodesic()
{
    if ( m_pGeodesic != NULL )
    {
        delete m_pGeodesic;
        m_pGeodesic = NULL;
    }
}

//*****************************************************************************
Geodesic::Geodesic(double a, double f)
{
    try
    {
        m_pGeodesic = new GeographicLib::Geodesic( a, f );
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
Geodesic::Geodesic()
{
    try
    {
        m_pGeodesic = new GeographicLib::Geodesic( GeographicLib::Geodesic::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
                    llat2, llon2, lazi2, lm12, lM12, lM21, lS12);
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
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
                    llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
    return out;
}

//*****************************************************************************
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
                    llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, lm12;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
                    llat2, llon2, lazi2, lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lM12, lM21;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
                    llat2, llon2, lazi2, lM12, lM21);
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double Geodesic::Direct(double lat1, double lon1, double azi1, double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesic->Direct(lat1, lon1, azi1, s12,
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
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
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
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12,
            llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12);
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
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2)
{
    double llat2, llon2;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12, llat2, llon2 );
    lat2 = llat2;
    lon2 = llon2;
}

//*****************************************************************************
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                   [System::Runtime::InteropServices::Out] double% lat2,
                   [System::Runtime::InteropServices::Out] double% lon2,
                   [System::Runtime::InteropServices::Out] double% azi2)
{
    double llat2, llon2, lazi2;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12, llat2, llon2, lazi2 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
}

//*****************************************************************************
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12)
{
    double llat2, llon2, lazi2, ls12;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12,
            llat2, llon2, lazi2, ls12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
}

//*****************************************************************************
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% m12)
{
    double llat2, llon2, lazi2, ls12, lm12;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12,
            llat2, llon2, lazi2, ls12, lm12 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
}

//*****************************************************************************
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% M12,
                [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lM12, lM21;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12,
            llat2, llon2, lazi2, ls12, lM12, lM21 );
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
void Geodesic::ArcDirect(double lat1, double lon1, double azi1, double a12,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% m12,
                [System::Runtime::InteropServices::Out] double% M12,
                [System::Runtime::InteropServices::Out] double% M21)
{
    double llat2, llon2, lazi2, ls12, lm12, lM12, lM21;
    m_pGeodesic->ArcDirect(lat1, lon1, azi1, a12,
            llat2, llon2, lazi2, ls12, lm12, lM12, lM21);
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
}

//*****************************************************************************
double Geodesic::GenDirect(double lat1, double lon1, double azi1,
                        bool arcmode, double s12_a12,
                        Geodesic::mask outmask,
                        [System::Runtime::InteropServices::Out] double% lat2,
                        [System::Runtime::InteropServices::Out] double% lon2,
                        [System::Runtime::InteropServices::Out] double% azi2,
                        [System::Runtime::InteropServices::Out] double% s12,
                        [System::Runtime::InteropServices::Out] double% m12,
                        [System::Runtime::InteropServices::Out] double% M12,
                        [System::Runtime::InteropServices::Out] double% M21,
                        [System::Runtime::InteropServices::Out] double% S12)
{
    double llat2, llon2, lazi2, lm12, lM12, lM21, ls12, lS12;
    double out = m_pGeodesic->GenDirect(lat1, lon1, azi1, arcmode, s12_a12,
                    static_cast<unsigned>(outmask),
                    llat2, llon2, lazi2, ls12, lm12, lM12, lM21, lS12);
    lat2 = llat2;
    lon2 = llon2;
    azi2 = lazi2;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    s12 = ls12;
    S12 = lS12;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2,
                    ls12, lazi1, lazi2, lm12, lM12, lM21, lS12);
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
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12)
{
    double ls12;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2, ls12 );
    s12 = ls12;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double lazi1, lazi2;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2, lazi1, lazi2);
    azi1 = lazi1;
    azi2 = lazi2;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2)
{
    double ls12, lazi1, lazi2;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2,
                    ls12, lazi1, lazi2 );
    azi1 = lazi1;
    azi2 = lazi2;
    s12 = ls12;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12)
{
    double ls12, lazi1, lazi2, lm12;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2,
                ls12, lazi1, lazi2, lm12 );
    azi1 = lazi1;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double ls12, lazi1, lazi2, lM12, lM21;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2,
                    ls12, lazi1, lazi2, lM12, lM21 );
    azi1 = lazi1;
    azi2 = lazi2;
    s12 = ls12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double Geodesic::Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21;
    double out = m_pGeodesic->Inverse(lat1, lon1, lat2, lon2,
                    ls12, lazi1, lazi2, lm12, lM12, lM21 );
    azi1 = lazi1;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    return out;
}

//*****************************************************************************
double Geodesic::GenInverse(double lat1, double lon1, double lat2, double lon2,
                        Geodesic::mask outmask,
                        [System::Runtime::InteropServices::Out] double% s12,
                        [System::Runtime::InteropServices::Out] double% azi1,
                        [System::Runtime::InteropServices::Out] double% azi2,
                        [System::Runtime::InteropServices::Out] double% m12,
                        [System::Runtime::InteropServices::Out] double% M12,
                        [System::Runtime::InteropServices::Out] double% M21,
                        [System::Runtime::InteropServices::Out] double% S12)
{
    double ls12, lazi1, lazi2, lm12, lM12, lM21, lS12;
    double out = m_pGeodesic->GenInverse(lat1, lon1, lat2, lon2,
                    static_cast<unsigned>(outmask),
                    ls12, lazi1, lazi2, lm12, lM12, lM21, lS12);
    azi1 = lazi1;
    azi2 = lazi2;
    s12 = ls12;
    m12 = lm12;
    M12 = lM12;
    M21 = lM21;
    S12 = lS12;
    return out;
}

//*****************************************************************************
System::IntPtr^ Geodesic::GetUnmanaged()
{
    return gcnew System::IntPtr( const_cast<void*>(reinterpret_cast<const void*>(m_pGeodesic)) );
}

//*****************************************************************************
GeodesicLine^ Geodesic::Line(double lat1, double lon1, double azi1,
                             NETGeographicLib::Mask caps )
{
    return gcnew GeodesicLine( this, lat1, lon1, azi1, caps );
}

//*****************************************************************************
GeodesicLine^ Geodesic::InverseLine(double lat1, double lon1, double lat2,
    double lon2, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLine(m_pGeodesic->InverseLine(lat1, lon1, lat2,
        lon2, static_cast<unsigned int>(caps)));
}

//*****************************************************************************
GeodesicLine^ Geodesic::DirectLine(double lat1, double lon1, double azi1,
    double s12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLine(m_pGeodesic->DirectLine(lat1, lon1, azi1,
        s12, static_cast<unsigned int>(caps)));
}

//*****************************************************************************
GeodesicLine^ Geodesic::ArcDirectLine(double lat1, double lon1, double azi1,
    double a12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLine(m_pGeodesic->ArcDirectLine(lat1, lon1, azi1,
        a12, static_cast<unsigned int>(caps)));
}

//*****************************************************************************
GeodesicLine^ Geodesic::GenDirectLine(double lat1, double lon1, double azi1,
    bool arcmode, double s12_a12, NETGeographicLib::Mask caps)
{
    return gcnew GeodesicLine(m_pGeodesic->GenDirectLine(lat1, lon1, azi1,
        arcmode, s12_a12, static_cast<unsigned int>(caps)));
}

//*****************************************************************************
double Geodesic::MajorRadius::get() { return m_pGeodesic->MajorRadius(); }

//*****************************************************************************
double Geodesic::Flattening::get() { return m_pGeodesic->Flattening(); }

//*****************************************************************************
double Geodesic::EllipsoidArea::get() { return m_pGeodesic->EllipsoidArea(); }
