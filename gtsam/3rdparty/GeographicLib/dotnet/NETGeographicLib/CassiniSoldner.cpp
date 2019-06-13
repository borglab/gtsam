/**
 * \file NETGeographicLib/CassiniSoldner.cpp
 * \brief Implementation for NETGeographicLib::CassiniSoldner class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/CassiniSoldner.hpp"
#include "Geodesic.h"
#include "CassiniSoldner.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::CassiniSoldner";

//*****************************************************************************
CassiniSoldner::!CassiniSoldner()
{
    if ( m_pCassiniSoldner != NULL )
    {
        delete m_pCassiniSoldner;
        m_pCassiniSoldner = NULL;
    }
}

//*****************************************************************************
CassiniSoldner::CassiniSoldner(double lat0, double lon0)
{
    try
    {
        m_pCassiniSoldner = new GeographicLib::CassiniSoldner( GeographicLib::Geodesic::WGS84() );
        m_pCassiniSoldner->Reset(lat0, lon0);
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
CassiniSoldner::CassiniSoldner(double lat0, double lon0,
                Geodesic^ earth )
{
    try
    {
        const GeographicLib::Geodesic* pGeodesic =
            reinterpret_cast<const GeographicLib::Geodesic*>(
                earth->GetUnmanaged()->ToPointer() );
        m_pCassiniSoldner = new GeographicLib::CassiniSoldner( *pGeodesic );
        m_pCassiniSoldner->Reset(lat0, lon0);
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void CassiniSoldner::Reset(double lat0, double lon0)
{
    m_pCassiniSoldner->Reset(lat0, lon0);
}

//*****************************************************************************
void CassiniSoldner::Forward(double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% azi,
                [System::Runtime::InteropServices::Out] double% rk)
{
    double lx, ly, lazi, lrk;
    m_pCassiniSoldner->Forward(lat, lon, lx, ly, lazi, lrk);
    x = lx;
    y = ly;
    azi = lazi;
    rk = lrk;
}

//*****************************************************************************
void CassiniSoldner::Reverse(double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% azi,
                [System::Runtime::InteropServices::Out] double% rk)
{
    double llat, llon, lazi, lrk;
    m_pCassiniSoldner->Reverse( x, y, llat, llon, lazi, lrk );
    lat = llat;
    lon = llon;
    azi = lazi;
    rk = lrk;
}
//*****************************************************************************
void CassiniSoldner::Forward(double lat, double lon,
    [System::Runtime::InteropServices::Out] double% x,
    [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly, azi, rk;
    m_pCassiniSoldner->Forward(lat, lon, lx, ly, azi, rk);
    x = lx;
    y = ly;
}

//*****************************************************************************
void CassiniSoldner::Reverse(double x, double y,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon, lazi, lrk;
    m_pCassiniSoldner->Reverse( x, y, llat, llon, lazi, lrk );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double CassiniSoldner::LatitudeOrigin::get()
{ return m_pCassiniSoldner->LatitudeOrigin(); }

//*****************************************************************************
double CassiniSoldner::LongitudeOrigin::get()
{ return m_pCassiniSoldner->LongitudeOrigin(); }

//*****************************************************************************
double CassiniSoldner::MajorRadius::get()
{ return m_pCassiniSoldner->MajorRadius(); }

//*****************************************************************************
double CassiniSoldner::Flattening::get()
{ return m_pCassiniSoldner->Flattening(); }
