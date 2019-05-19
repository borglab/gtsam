/**
 * \file NETGeographicLib/Geohash.cpp
 * \brief Implementation for NETGeographicLib::Geohash class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Geohash.hpp"
#include "Geohash.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
void Geohash::Forward(double lat, double lon, int len, System::String^% geohash)
{
    try
    {
        std::string l;
        GeographicLib::Geohash::Forward( lat, lon, len, l );
        geohash = gcnew System::String( l.c_str() );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
void Geohash::Reverse(System::String^ geohash,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon,
    [System::Runtime::InteropServices::Out] int% len,
    bool centerp )
{
    try
    {
        double llat, llon;
        int llen;
        GeographicLib::Geohash::Reverse( StringConvert::ManagedToUnmanaged( geohash ),
            llat, llon, llen, centerp );
        lat = llat;
        lon = llon;
        len = llen;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double Geohash::LatitudeResolution(int len)
{
    return GeographicLib::Geohash::LatitudeResolution( len );
}

//*****************************************************************************
double Geohash::LongitudeResolution(int len)
{
    return GeographicLib::Geohash::LongitudeResolution( len );
}

//*****************************************************************************
int Geohash::GeohashLength(double res)
{
    return GeographicLib::Geohash::GeohashLength( res );
}

//*****************************************************************************
int Geohash::GeohashLength(double latres, double lonres)
{
    return GeographicLib::Geohash::GeohashLength( latres, lonres );
}

//*****************************************************************************
int Geohash::DecimalPrecision(int len)
{
    return GeographicLib::Geohash::DecimalPrecision( len );
}
