/**
 * \file NETGeographicLib/AlbersEqualArea.cpp
 * \brief Implementation for NETGeographicLib::AlbersEqualArea class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/AlbersEqualArea.hpp"
#include "AlbersEqualArea.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
AlbersEqualArea::!AlbersEqualArea()
{
    if ( m_pAlbersEqualArea != NULL )
    {
        delete m_pAlbersEqualArea;
        m_pAlbersEqualArea = NULL;
    }
}

//*****************************************************************************
AlbersEqualArea::AlbersEqualArea( StandardTypes type )
{
    try
    {
        switch ( type )
        {
        case StandardTypes::CylindricalEqualArea:
            m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea( GeographicLib::AlbersEqualArea::CylindricalEqualArea() );
            break;
        case StandardTypes::AzimuthalEqualAreaNorth:
            m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea( GeographicLib::AlbersEqualArea::AzimuthalEqualAreaNorth() );
            break;
        case StandardTypes::AzimuthalEqualAreaSouth:
            m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea( GeographicLib::AlbersEqualArea::AzimuthalEqualAreaSouth() );
            break;
        }
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::AlbersEqualArea" );
    }
}

//*****************************************************************************
AlbersEqualArea::AlbersEqualArea(double a, double f, double stdlat1, double stdlat2, double k1)
{
    try
    {
        m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea( a, f, stdlat1, stdlat2, k1 );
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::AlbersEqualArea" );
    }
}

//*****************************************************************************
AlbersEqualArea::AlbersEqualArea(double a, double f, double stdlat, double k0)
{
    try
    {
        m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea( a, f, stdlat, k0 );
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::AlbersEqualArea" );
    }
}

//*****************************************************************************
AlbersEqualArea::AlbersEqualArea(double a, double f,
                        double sinlat1, double coslat1,
                        double sinlat2, double coslat2,
                        double k1)
{
    try
    {
        m_pAlbersEqualArea = new GeographicLib::AlbersEqualArea(
            a, f, sinlat1, coslat1, sinlat2, coslat2, k1 );
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::AlbersEqualArea" );
    }
}

//*****************************************************************************
void AlbersEqualArea::SetScale(double lat, double k)
{
    try
    {
        m_pAlbersEqualArea->SetScale( lat, k );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
void AlbersEqualArea::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    m_pAlbersEqualArea->Forward( lon0, lat, lon, lx, ly, lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void AlbersEqualArea::Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    m_pAlbersEqualArea->Reverse( lon0, x, y, llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void AlbersEqualArea::Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly, lgamma, lk;
    m_pAlbersEqualArea->Forward( lon0, lat, lon, lx, ly, lgamma, lk );
    x = lx;
    y = ly;
}

//*****************************************************************************
void AlbersEqualArea::Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon, lgamma, lk;
    m_pAlbersEqualArea->Reverse( lon0, x, y, llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double AlbersEqualArea::MajorRadius::get()
{ return m_pAlbersEqualArea->MajorRadius(); }

//*****************************************************************************
double AlbersEqualArea::Flattening::get()
{ return m_pAlbersEqualArea->Flattening(); }

//*****************************************************************************
double AlbersEqualArea::OriginLatitude::get()
{ return m_pAlbersEqualArea->OriginLatitude(); }

//*****************************************************************************
double AlbersEqualArea::CentralScale::get()
{ return m_pAlbersEqualArea->CentralScale(); }
