/**
 * \file NETGeographicLib/Geocentric.cpp
 * \brief Implementation for NETGeographicLib::Geocentric class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Geocentric.hpp"
#include "Geocentric.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Unable to allocate memory for a GeographicLib::Geocentric";

//*****************************************************************************
Geocentric::!Geocentric()
{
    if ( m_pGeocentric != NULL )
    {
        delete m_pGeocentric;
        m_pGeocentric = NULL;
    }
}

//*****************************************************************************
Geocentric::Geocentric(void)
{
    try
    {
        m_pGeocentric = new GeographicLib::Geocentric(
            GeographicLib::Geocentric::WGS84() );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
Geocentric::Geocentric(double a, double f)
{
    try
    {
        m_pGeocentric = new GeographicLib::Geocentric( a, f );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch (std::exception err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
Geocentric::Geocentric( const GeographicLib::Geocentric& g )
{
    try
    {
        m_pGeocentric = new GeographicLib::Geocentric( g );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void Geocentric::Forward(double lat, double lon, double h,
    [System::Runtime::InteropServices::Out] double% X,
    [System::Runtime::InteropServices::Out] double% Y,
    [System::Runtime::InteropServices::Out] double% Z)
{
    double lX, lY, lZ;
    m_pGeocentric->Forward( lat, lon, h, lX, lY, lZ);
    X = lX;
    Y = lY;
    Z = lZ;
}

//*****************************************************************************
void Geocentric::Forward(double lat, double lon, double h,
    [System::Runtime::InteropServices::Out] double% X,
    [System::Runtime::InteropServices::Out] double% Y,
    [System::Runtime::InteropServices::Out] double% Z,
    [System::Runtime::InteropServices::Out] array<double,2>^% M)
{
    double lX, lY, lZ;
    std::vector<double> lM(9);
    m_pGeocentric->Forward( lat, lon, h, lX, lY, lZ, lM);
    X = lX;
    Y = lY;
    Z = lZ;
    M = gcnew array<double,2>( 3, 3 );
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            M[i,j] = lM[3*i+j];
}

//*****************************************************************************
void Geocentric::Reverse(double X, double Y, double Z,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon,
    [System::Runtime::InteropServices::Out] double% h)
{
    double llat, llon, lh;
    m_pGeocentric->Reverse(X, Y, Z, llat, llon, lh);
    lat = llat;
    lon = llon;
    h = lh;
}

//*****************************************************************************
void Geocentric::Reverse(double X, double Y, double Z,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon,
    [System::Runtime::InteropServices::Out] double% h,
    [System::Runtime::InteropServices::Out] array<double,2>^% M)
{
    std::vector<double> lM(9);
    double llat, llon, lh;
    m_pGeocentric->Reverse(X, Y, Z, llat, llon, lh, lM);
    lat = llat;
    lon = llon;
    h = lh;
    M = gcnew array<double,2>( 3, 3 );
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            M[i,j] = lM[3*i+j];
}

//*****************************************************************************
System::IntPtr^ Geocentric::GetUnmanaged()
{
    return gcnew System::IntPtr( const_cast<void*>(
        reinterpret_cast<const void*>(m_pGeocentric) ) );
}

//*****************************************************************************
double Geocentric::MajorRadius::get()
{ return m_pGeocentric->MajorRadius(); }

//*****************************************************************************
double Geocentric::Flattening::get()
{ return m_pGeocentric->Flattening(); }
