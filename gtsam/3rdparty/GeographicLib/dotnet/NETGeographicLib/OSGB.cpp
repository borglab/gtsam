/**
 * \file NETGeographicLib/OSGB.cpp
 * \brief Implementation for NETGeographicLib::OSGB class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/OSGB.hpp"
#include "OSGB.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
void OSGB::Forward(double lat, double lon,
                    [System::Runtime::InteropServices::Out] double% x,
                    [System::Runtime::InteropServices::Out] double% y,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    GeographicLib::OSGB::Forward( lat, lon, lx, ly, lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void OSGB::Reverse(double x, double y,
                    [System::Runtime::InteropServices::Out] double% lat,
                    [System::Runtime::InteropServices::Out] double% lon,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    GeographicLib::OSGB::Reverse( x, y, llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void OSGB::Forward(double lat, double lon,
    [System::Runtime::InteropServices::Out] double% x,
    [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    GeographicLib::OSGB::Forward( lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void OSGB::Reverse(double x, double y,
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    GeographicLib::OSGB::Reverse( x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
void OSGB::GridReference(double x, double y, int prec,
    [System::Runtime::InteropServices::Out] System::String^% gridref)
{
    try
    {
        std::string lgridref;
        GeographicLib::OSGB::GridReference( x, y, prec, lgridref );
        gridref = gcnew System::String( lgridref.c_str() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Memory allocation error in OSGB::GridReference" );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void OSGB::GridReference(System::String^ gridref,
                    [System::Runtime::InteropServices::Out] double% x,
                    [System::Runtime::InteropServices::Out] double% y,
                    [System::Runtime::InteropServices::Out] int% prec,
                    bool centerp )
{
    try
    {
        double lx, ly;
        int lprec;
        GeographicLib::OSGB::GridReference(
            StringConvert::ManagedToUnmanaged( gridref ),
            lx, ly, lprec, centerp );
        x = lx;
        y = ly;
        prec = lprec;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double OSGB::MajorRadius() { return GeographicLib::OSGB::MajorRadius(); }

//*****************************************************************************
double OSGB::Flattening() { return GeographicLib::OSGB::Flattening(); }

//*****************************************************************************
double OSGB::CentralScale() { return GeographicLib::OSGB::CentralScale(); }

//*****************************************************************************
double OSGB::OriginLatitude() { return GeographicLib::OSGB::OriginLatitude(); }

//*****************************************************************************
double OSGB::OriginLongitude()
{ return GeographicLib::OSGB::OriginLongitude(); }

//*****************************************************************************
double OSGB::FalseNorthing() { return GeographicLib::OSGB::FalseNorthing(); }

//*****************************************************************************
double OSGB::FalseEasting() { return GeographicLib::OSGB::FalseEasting(); }
