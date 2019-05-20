/**
 * \file NETGeographicLib/LambertConformalConic.cpp
 * \brief Implementation for NETGeographicLib::LambertConformalConic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/LambertConformalConic.hpp"
#include "LambertConformalConic.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocte memory for a GeographicLib::LambertConformalConic";

//*****************************************************************************
LambertConformalConic::!LambertConformalConic(void)
{
    if ( m_pLambertConformalConic != NULL )
    {
        delete m_pLambertConformalConic;
        m_pLambertConformalConic = NULL;
    }
}

//*****************************************************************************
LambertConformalConic::LambertConformalConic(double a, double f, double stdlat,
                                             double k0)
{
    try
    {
        m_pLambertConformalConic =
            new GeographicLib::LambertConformalConic( a, f, stdlat, k0 );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
LambertConformalConic::LambertConformalConic(double a, double f,
    double stdlat1, double stdlat2, double k1)
{
    try
    {
        m_pLambertConformalConic =
            new GeographicLib::LambertConformalConic( a, f, stdlat1, stdlat2, k1 );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
LambertConformalConic::LambertConformalConic(double a, double f,
                        double sinlat1, double coslat1,
                        double sinlat2, double coslat2,
                        double k1)
{
    try
    {
        m_pLambertConformalConic =
            new GeographicLib::LambertConformalConic( a, f, sinlat1, coslat1,
                        sinlat2, coslat2, k1 );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
LambertConformalConic::LambertConformalConic()
{
    try
    {
        m_pLambertConformalConic = new GeographicLib::LambertConformalConic(
            GeographicLib::LambertConformalConic::Mercator() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void LambertConformalConic::SetScale(double lat, double k)
{
    try
    {
        m_pLambertConformalConic->SetScale( lat, k );
    }
    catch ( std::exception err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void LambertConformalConic::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    m_pLambertConformalConic->Forward( lon0, lat, lon, lx, ly, lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void LambertConformalConic::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    m_pLambertConformalConic->Reverse( lon0, x, y, llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void LambertConformalConic::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    m_pLambertConformalConic->Forward( lon0, lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void LambertConformalConic::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pLambertConformalConic->Reverse( lon0, x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double LambertConformalConic::MajorRadius::get()
{ return m_pLambertConformalConic->MajorRadius(); }

//*****************************************************************************
double LambertConformalConic::Flattening::get()
{ return m_pLambertConformalConic->Flattening(); }

//*****************************************************************************
double LambertConformalConic::OriginLatitude::get()
{ return m_pLambertConformalConic->OriginLatitude(); }

//*****************************************************************************
double LambertConformalConic::CentralScale::get()
{ return m_pLambertConformalConic->CentralScale(); }
