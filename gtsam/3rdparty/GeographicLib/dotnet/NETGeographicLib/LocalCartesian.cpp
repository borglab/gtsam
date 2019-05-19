/**
 * \file NETGeographicLib/LocalCartesian.cpp
 * \brief Implementation for NETGeographicLib::LocalCartesian class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "LocalCartesian.h"
#include "Geocentric.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::LocalCartesian";

//*****************************************************************************
LocalCartesian::!LocalCartesian(void)
{
    if ( m_pLocalCartesian != NULL )
    {
        delete m_pLocalCartesian;
        m_pLocalCartesian = NULL;
    }
}

//*****************************************************************************
LocalCartesian::LocalCartesian(double lat0, double lon0, double h0,
                Geocentric^ earth )
{
    try
    {
        const GeographicLib::Geocentric* pGeocentric =
            reinterpret_cast<const GeographicLib::Geocentric*>(
                earth->GetUnmanaged()->ToPointer() );
        m_pLocalCartesian = new GeographicLib::LocalCartesian( lat0, lon0, h0, *pGeocentric );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
LocalCartesian::LocalCartesian(double lat0, double lon0, double h0 )
{
    try
    {
        m_pLocalCartesian = new GeographicLib::LocalCartesian( lat0, lon0, h0,
            GeographicLib::Geocentric::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
LocalCartesian::LocalCartesian(Geocentric^ earth)
{
    try
    {
        const GeographicLib::Geocentric* pGeocentric =
            reinterpret_cast<const GeographicLib::Geocentric*>(
                earth->GetUnmanaged()->ToPointer() );
        m_pLocalCartesian = new GeographicLib::LocalCartesian( *pGeocentric );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
LocalCartesian::LocalCartesian()
{
    try
    {
        m_pLocalCartesian = new GeographicLib::LocalCartesian(
            GeographicLib::Geocentric::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void LocalCartesian::Reset(double lat0, double lon0, double h0 )
{
    m_pLocalCartesian->Reset( lat0, lon0, h0 );
}

//*****************************************************************************
void LocalCartesian::Forward(double lat, double lon, double h,
    [System::Runtime::InteropServices::Out] double% x,
    [System::Runtime::InteropServices::Out] double% y,
    [System::Runtime::InteropServices::Out] double% z)
{
    double lx, ly, lz;
    m_pLocalCartesian->Forward( lat, lon, h, lx, ly, lz );
    x = lx;
    y = ly;
    z = lz;
}

//*****************************************************************************
void LocalCartesian::Forward(double lat, double lon, double h,
    [System::Runtime::InteropServices::Out] double% x,
    [System::Runtime::InteropServices::Out] double% y,
    [System::Runtime::InteropServices::Out] double% z,
    [System::Runtime::InteropServices::Out] array<double,2>^% M)
{
    double lx, ly, lz;
    std::vector<double> lM(9);
    m_pLocalCartesian->Forward( lat, lon, h, lx, ly, lz, lM );
    x = lx;
    y = ly;
    z = lz;
    M = gcnew array<double,2>(3,3);
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            M[i,j] = lM[3*i+j];
}

//*****************************************************************************
void LocalCartesian::Reverse(double x, double y, double z,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon,
    [System::Runtime::InteropServices::Out] double% h)
{
    double llat, llon, lh;
    m_pLocalCartesian->Reverse( x, y, z, llat, llon, lh );
    lat = llat;
    lon = llon;
    h = lh;
}

//*****************************************************************************
void LocalCartesian::Reverse(double x, double y, double z,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon,
    [System::Runtime::InteropServices::Out] double% h,
    [System::Runtime::InteropServices::Out] array<double,2>^% M)
{
    double llat, llon, lh;
    std::vector<double> lM(9);
    m_pLocalCartesian->Reverse( x, y, z, llat, llon, lh, lM );
    lat = llat;
    lon = llon;
    h = lh;
    M = gcnew array<double,2>(3,3);
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            M[i,j] = lM[3*i+j];
}

//*****************************************************************************
double LocalCartesian::LatitudeOrigin::get()
{ return m_pLocalCartesian->LatitudeOrigin(); }

//*****************************************************************************
double LocalCartesian::LongitudeOrigin::get()
{ return m_pLocalCartesian->LongitudeOrigin(); }

//*****************************************************************************
double LocalCartesian::HeightOrigin::get()
{ return m_pLocalCartesian->HeightOrigin(); }

//*****************************************************************************
double LocalCartesian::MajorRadius::get()
{ return m_pLocalCartesian->MajorRadius(); }

//*****************************************************************************
double LocalCartesian::Flattening::get()
{ return m_pLocalCartesian->Flattening(); }
