/**
 * \file NETGeographicLib/Gnomonic.cpp
 * \brief Implementation for NETGeographicLib::Gnomonic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Gnomonic.hpp"
#include "Gnomonic.h"
#include "Geodesic.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::Gnomonic";

//*****************************************************************************
Gnomonic::!Gnomonic(void)
{
    if ( m_pGnomonic != NULL )
    {
        delete m_pGnomonic;
        m_pGnomonic = NULL;
    }
}

//*****************************************************************************
Gnomonic::Gnomonic( Geodesic^ earth )
{
    try
    {
        const GeographicLib::Geodesic* pGeodesic =
            reinterpret_cast<const GeographicLib::Geodesic*>(
                earth->GetUnmanaged()->ToPointer() );
        m_pGnomonic = new GeographicLib::Gnomonic( *pGeodesic );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
Gnomonic::Gnomonic()
{
    try
    {
        m_pGnomonic = new GeographicLib::Gnomonic( GeographicLib::Geodesic::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void Gnomonic::Forward(double lat0, double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% azi,
                [System::Runtime::InteropServices::Out] double% rk)
{
    double lx, ly, lazi, lrk;
    m_pGnomonic->Forward( lat0, lon0, lat, lon, lx, ly, lazi, lrk );
    x = lx;
    y = ly;
    azi = lazi;
    rk = lrk;
}

//*****************************************************************************
void Gnomonic::Reverse(double lat0, double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% azi,
                [System::Runtime::InteropServices::Out] double% rk)
{
    double llat, llon, lazi, lrk;
    m_pGnomonic->Reverse( lat0, lon0, x, y, llat, llon, lazi, lrk );
    lat = llat;
    lon = llon;
    azi = lazi;
    rk = lrk;
}

//*****************************************************************************
void Gnomonic::Forward(double lat0, double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    m_pGnomonic->Forward( lat0, lon0, lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void Gnomonic::Reverse(double lat0, double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pGnomonic->Reverse( lat0, lon0, x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double Gnomonic::MajorRadius::get() { return m_pGnomonic->MajorRadius(); }

//*****************************************************************************
double Gnomonic::Flattening::get() { return m_pGnomonic->Flattening(); }
