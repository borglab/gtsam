/**
 * \file NETGeographicLib/UTMUPS.cpp
 * \brief Implementation for NETGeographicLib::UTMUPS class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/UTMUPS.hpp"
#include "UTMUPS.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
int UTMUPS::StandardZone(double lat, double lon, int setzone)
{
    try
    {
        return GeographicLib::UTMUPS::StandardZone( lat, lon, setzone );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::Forward(double lat, double lon,
                    [System::Runtime::InteropServices::Out] int% zone,
                    [System::Runtime::InteropServices::Out] bool% northp,
                    [System::Runtime::InteropServices::Out] double% x,
                    [System::Runtime::InteropServices::Out] double% y,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k,
                    int setzone, bool mgrslimits)
{
    try
    {
        int lzone;
        bool lnorthp;
        double lx, ly, lgamma, lk;
        GeographicLib::UTMUPS::Forward(lat, lon,
                                       lzone, lnorthp, lx, ly,
                                       lgamma, lk,
                                       setzone, mgrslimits);
        zone = lzone;
        northp = northp;
        x = lx;
        y = ly;
        gamma = lgamma;
        k = lk;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::Reverse(int zone, bool northp, double x, double y,
                    [System::Runtime::InteropServices::Out] double% lat,
                    [System::Runtime::InteropServices::Out] double% lon,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k,
                    bool mgrslimits)
{
    try
    {
        double llat, llon, lgamma, lk;
        GeographicLib::UTMUPS::Reverse( zone, northp, x, y, llat, llon, lgamma,
                                        lk, mgrslimits );
        lat = llat;
        lon = llon;
        gamma = lgamma;
        k = lk;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::Forward(double lat, double lon,
                    [System::Runtime::InteropServices::Out] int% zone,
                    [System::Runtime::InteropServices::Out] bool% northp,
                    [System::Runtime::InteropServices::Out] double% x,
                    [System::Runtime::InteropServices::Out] double% y,
                    int setzone, bool mgrslimits )
{
    try
    {
        double gamma, k, lx, ly;
        bool lnorthp;
        int lzone;
        GeographicLib::UTMUPS::Forward(lat, lon, lzone, lnorthp, lx, ly,
                                       gamma, k, setzone, mgrslimits);
        x = lx;
        y = ly;
        zone = lzone;
        northp = lnorthp;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::Reverse(int zone, bool northp, double x, double y,
                    [System::Runtime::InteropServices::Out] double% lat,
                    [System::Runtime::InteropServices::Out] double% lon,
                    bool mgrslimits)
{
    try
    {
        double gamma, k, llat, llon;
        GeographicLib::UTMUPS::Reverse(zone, northp, x, y, llat, llon, gamma,
                                        k, mgrslimits);
        lat = llat;
        lon = llon;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::Transfer(int zonein, bool northpin, double xin, double yin,
                        int zoneout, bool northpout,
                        [System::Runtime::InteropServices::Out] double% xout,
                        [System::Runtime::InteropServices::Out] double% yout,
                        [System::Runtime::InteropServices::Out] int% zone)
{
    try
    {
        int lzone;
        double lxout, lyout;
        GeographicLib::UTMUPS::Transfer(zonein, northpin, xin, yin,
                                        zoneout, northpout, lxout, lyout,
                                        lzone);
        xout = lxout;
        yout = lyout;
        zone = lzone;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::DecodeZone(System::String^ zonestr,
    [System::Runtime::InteropServices::Out] int% zone,
    [System::Runtime::InteropServices::Out] bool% northp)
{
    try
    {
        std::string zoneIn = StringConvert::ManagedToUnmanaged( zonestr );
        int lzone;
        bool lnorthp;
        GeographicLib::UTMUPS::DecodeZone( zoneIn, lzone, lnorthp );
        zone = lzone;
        northp = lnorthp;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ UTMUPS::EncodeZone(int zone, bool northp, bool abbrev)
{
    try
    {
        return StringConvert::UnmanagedToManaged( GeographicLib::UTMUPS::EncodeZone( zone, northp, abbrev ) );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void UTMUPS::DecodeEPSG(int epsg,
    [System::Runtime::InteropServices::Out] int% zone,
    [System::Runtime::InteropServices::Out] bool% northp)
{
    int lzone;
    bool lnorthp;
    GeographicLib::UTMUPS::DecodeEPSG( epsg, lzone, lnorthp );
    zone = lzone;
    northp = lnorthp;
}

//*****************************************************************************
int UTMUPS::EncodeEPSG(int zone, bool northp)
{
    return GeographicLib::UTMUPS::EncodeEPSG( zone, northp );
}

//****************************************************************************
double UTMUPS::UTMShift() { return GeographicLib::UTMUPS::UTMShift(); }

//****************************************************************************
double UTMUPS::MajorRadius() { return GeographicLib::UTMUPS::MajorRadius(); }

//****************************************************************************
double UTMUPS::Flattening() { return GeographicLib::UTMUPS::Flattening(); }
