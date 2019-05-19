/**
 * \file NETGeographicLib/AzimuthalEquidistant.cpp
 * \brief Implementation for NETGeographicLib::AzimuthalEquidistant class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/AzimuthalEquidistant.hpp"
#include "AzimuthalEquidistant.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::AzimuthalEquidistant";

//*****************************************************************************
AzimuthalEquidistant::!AzimuthalEquidistant()
{
    if ( m_pAzimuthalEquidistant != NULL )
    {
        delete m_pAzimuthalEquidistant;
        m_pAzimuthalEquidistant = NULL;
    }
}

//*****************************************************************************
AzimuthalEquidistant::AzimuthalEquidistant(void)
{
    try
    {
        m_pAzimuthalEquidistant = new GeographicLib::AzimuthalEquidistant( GeographicLib::Geodesic::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
AzimuthalEquidistant::AzimuthalEquidistant( Geodesic^ g )
{
    try
    {
        const GeographicLib::Geodesic* pGeodesic =
            reinterpret_cast<const GeographicLib::Geodesic*>(
                g->GetUnmanaged()->ToPointer() );
        m_pAzimuthalEquidistant = new GeographicLib::AzimuthalEquidistant( *pGeodesic );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void AzimuthalEquidistant::Forward(double lat0, double lon0, double lat, double lon,
                double% x, double% y, double% azi, double% rk)
{
    double lx, ly, lazi, lrk;
    m_pAzimuthalEquidistant->Forward( lat0, lon0, lat, lon,
            lx, ly, lazi, lrk );
    x = lx;
    y = ly;
    azi = lazi;
    rk = lrk;
}

//*****************************************************************************
void AzimuthalEquidistant::Reverse(double lat0, double lon0, double x, double y,
                double% lat, double% lon, double% azi, double% rk)
{
    double llat, llon, lazi, lrk;
    m_pAzimuthalEquidistant->Reverse(lat0, lon0, x, y,
            llat, llon, lazi, lrk);
    lat = llat;
    lon = llon;
    azi = lazi;
    rk = lrk;
}
//*****************************************************************************
void AzimuthalEquidistant::Forward(double lat0, double lon0, double lat, double lon,
                double% x, double% y)
{
    double azi, rk, lx, ly;
    m_pAzimuthalEquidistant->Forward(lat0, lon0, lat, lon, lx, ly, azi, rk);
    x = lx;
    y = ly;
}

//*****************************************************************************
void AzimuthalEquidistant::Reverse(double lat0, double lon0, double x, double y,
                double% lat, double% lon)
{
    double azi, rk, llat, llon;
    m_pAzimuthalEquidistant->Reverse(lat0, lon0, x, y, llat, llon, azi, rk);
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double AzimuthalEquidistant::MajorRadius::get()
{ return m_pAzimuthalEquidistant->MajorRadius(); }

//*****************************************************************************
double AzimuthalEquidistant::Flattening::get()
{ return m_pAzimuthalEquidistant->Flattening(); }
